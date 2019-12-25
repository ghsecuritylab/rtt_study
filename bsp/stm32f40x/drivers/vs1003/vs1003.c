//文件名：vs1003.c
//功能：vs1003 的底层驱动程序，主控芯片为msp340-149；
//     其他的微处理器(带SPI接口的)只需稍加修改即可适用；

#include "vs1003.h"
#include <string.h>

#define vs1003_printf rt_kprintf
#define CS_DEVICE_NAME "spi20"
#define DCS_DEVICE_NAME "spi21"
#define VS_1003_BUFF_LEN 4*1024

struct rt_spi_device *vs1003_cs_device = NULL;//命令设备
struct rt_spi_device *vs1003_dcs_device = NULL;//数据设备
//后面如果考虑性能，可以将命令传输直接操作spi,因为交替操作比较频繁的话，
//两个设备在传输的时候还要重新配置spi，虽然他们的配置都是相同的
void vs_spi_dma_stop();

static u8 vs1003_data_buff[VS_1003_BUFF_LEN];
static u8 spi_dma_status = 0;
u8 vs_dma_status(void)
{
    return spi_dma_status;
}
void vs1003_spi_dma_init(void)
{
    DMA_InitTypeDef            DMA_InitStructure;
    NVIC_InitTypeDef				NVIC_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);//打开外设时钟DMAspi2 tx流4

    //SPI2 TX DMA配置DMA1 流4,0通道 
    DMA_DeInit(DMA1_Stream4);//清空之前该stream4上的所有中断标志
    while (DMA_GetCmdStatus(DMA1_Stream4) != DISABLE){}//等待DMA可配置 
    
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;  //通道选择
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&(SPI2->DR);
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)vs1003_data_buff;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;//内存到外设
	DMA_InitStructure.DMA_BufferSize = VS_1003_BUFF_LEN;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设地址自动增加屏蔽
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//内存自动增加打开
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//周期模式，取完4K数据后，再从起始位置开始取，循环无休止
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//存储器突发单次传输
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//外设突发单次传输
	DMA_Init(DMA1_Stream4, &DMA_InitStructure);
	
	DMA_ClearITPendingBit(DMA1_Stream4,DMA_IT_TCIF5); //传输完成中断悬挂位
	DMA_ClearITPendingBit(DMA1_Stream4,DMA_IT_HTIF5);//传输一半完成中断悬挂位
	DMA_ITConfig(DMA1_Stream4,DMA_IT_TC,ENABLE);//打开全部传输完整中断也就是4K数据
	DMA_ITConfig(DMA1_Stream4,DMA_IT_HT,ENABLE);//打开传输完一半数据中断，也就是传输完2K
	
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream4_IRQn;//设置DMA中断
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);
	DMA_Cmd(DMA1_Stream4, ENABLE);//打开该SPI的控制器流
	vs_spi_dma_stop();
}

void vs_spi_dma_stop()
{
    SPI_I2S_DMACmd(SPI2,SPI_I2S_DMAReq_Tx,DISABLE);
    spi_dma_status = 0;
}
void vs_spi_dma_start()
{
    SPI_I2S_DMACmd(SPI2,SPI_I2S_DMAReq_Tx,ENABLE);
    spi_dma_status = 1;
}

void pc6_gpio_int_handler(void)
{
    vs1003_printf("pc6_gpio_int_handler\n");
    vs_spi_dma_stop();
}

void DMA1_Stream4_IRQHandler(void)
{
    rt_interrupt_enter();
    //rt_sem_release(dma_int);
    //DataRequestFlag = 1;
	if(DMA_GetITStatus(DMA1_Stream4,DMA_IT_TCIF4))//传输完成中断标志
    {
       // DataRequestFlag = 1;
    }
	else if(DMA_GetITStatus(DMA1_Stream5,DMA_IT_HTIF5))//传输一半完成中断标志
	{
       // DataRequestFlag = 2;
    }
	DMA_ClearITPendingBit(DMA1_Stream5,DMA_IT_TCIF5 | DMA_IT_HTIF5);
	rt_interrupt_leave();
}

void vs1003_spi_init(void)
{ 
    #if 0 /*xqy 2019-12-25*/
    vs1003_cs_device = (struct rt_spi_device *)rt_device_find(CS_DEVICE_NAME);
    if(!vs1003_cs_device)
    {
        vs1003_printf("not find %s\n",CS_DEVICE_NAME);
        return ;
    }
    vs1003_dcs_device = (struct rt_spi_device *)rt_device_find(DCS_DEVICE_NAME);
    if(!vs1003_cs_device)
    {
        vs1003_printf("not find %s\n",DCS_DEVICE_NAME);
        return ;
    }
    {
        struct rt_spi_configuration cfg;
        cfg.data_width = 8;
        cfg.mode = RT_SPI_MODE_0 | RT_SPI_MSB; /* SPI Compatible: Mode 0 and Mode 3 */
        cfg.max_hz = 50 * 1000 * 1000; /* 50M */
        rt_spi_configure(vs1003_cs_device, &cfg);
        rt_spi_configure(vs1003_dcs_device, &cfg);
    }
    #endif
    GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef SPI_InitStructure;

    rt_memset(vs1003_data_buff,0,VS_1003_BUFF_LEN);//清空BUFF数据区
    /* Enable GPIO clock */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;//
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_2;
    GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化
    
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_SPI2); 
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource2,GPIO_AF_SPI2); 
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource3,GPIO_AF_SPI2); 
 
	//这里只针对SPI口初始化
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2,ENABLE);//复位SPI2
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2,DISABLE);//停止复位SPI2

    SPI_StructInit(&SPI_InitStructure);
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    /* baudrate *///外设总线频率为84MHZ，32分频，则再除以32可以得到SPI主频
    //先往低的来，怕vs1003接收不来，OK了再升频率
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    /* CPHA */
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    /* MSB or LSB */
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;//全双工
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_NSS  = SPI_NSS_Soft;//cs脚软控
    
    /* init SPI */
    SPI_I2S_DeInit(SPI2);
    SPI_Init(SPI2, &SPI_InitStructure);
    /* Enable SPI_MASTER */
    SPI_Cmd(SPI2, ENABLE);
    SPI_CalculateCRC(SPI2, DISABLE);
}
/**********************************************************/
/*  函数名称 :  InitPortVS1003                            */
/*  函数功能 ： MCU与vs1003接口的初始化                   */
/*  参数     :  无                                        */
/*  返回值   :  无                                        */
/*--------------------------------------------------------*/
/*stm32f4开发板与vs1003的接口定义
#define VS_XRESET      PB12
#define VS_MISO        PC2   
#define VS_MOSI        PC3   
#define VS_SCLK        PB13 
#define VS_DREQ        PC6
#define VS_XCS         PC13  
#define VS_XDCS        PC0
*/
void  vs_io_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef   NVIC_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
	
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//使能SYSCFG时钟
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;	//VS_XRESET
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;//xcs命令控制片选
    GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化
    GPIO_SetBits(GPIOC, GPIO_Pin_13);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;//xdcs数据控制片选
    GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化
    GPIO_SetBits(GPIOC, GPIO_Pin_0);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;//VS_DREQ
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化

    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC,GPIO_PinSource6);
    /* 配置EXTI_Line11 */
    EXTI_InitStructure.EXTI_Line = EXTI_Line6;//LINE11
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//中断事件
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //下降沿触发 
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;//使能LINE0
    EXTI_Init(&EXTI_InitStructure);//配置

	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;//外部中断0
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//抢占优先级0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;//子优先级2
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
    NVIC_Init(&NVIC_InitStructure);//配置
}
s32 vs_dreq_status(void)//vs1003是否还有空间接收数据，高电平:有空间，低电平:无空间
{
    return GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_6);
}
void vs_reset(u8 status)
{
    if(status)
    {
        GPIO_SetBits(GPIOB, GPIO_Pin_12);
    }
    else
    {
        GPIO_ResetBits(GPIOB, GPIO_Pin_12);
    }
}
void xcs_cs(u8 status)
{
    if(status)
    {
        GPIO_SetBits(GPIOC, GPIO_Pin_13);
    }
    else
    {
        if(vs_dma_status())
        {
            vs_spi_dma_stop();
        }
        GPIO_SetBits(GPIOC, GPIO_Pin_0);
        GPIO_ResetBits(GPIOC, GPIO_Pin_13);
    }
}
void xdcs_cs(u8 status)
{
    if(status)
    {
        GPIO_SetBits(GPIOC, GPIO_Pin_0);
    }
    else
    {
        GPIO_SetBits(GPIOC, GPIO_Pin_13);
        GPIO_ResetBits(GPIOC, GPIO_Pin_0);
        
    }
}

/**********************************************************/
/*  函数名称 :  SPIPutChar                                */
/*  函数功能 ： 通过SPI发送一个字节的数据                 */
/*  参数     :  待发送的字节数据                          */
/*  返回值   :  无                                        */
/*--------------------------------------------------------*/

unsigned char SPI_Write_Byte_vs(unsigned char byte)
{ 
    unsigned char temp=0;
    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
    SPI_I2S_SendData(SPI2,byte);
    
    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
    // Get the received data
    temp = SPI_I2S_ReceiveData(SPI2);
    return temp;
}


/******************************************************************
 - 功能描述：向VS1003的功能寄存器中写入数据（一个字，即两个字节）
 - 隶属模块：VS1003B模块
 - 函数属性：外部，用户可调用
 - 参数说明：addr是功能寄存器的地址
             hdat是要写入的高字节
             ldat是要写入的低字节
 - 返回说明：无返回
 ******************************************************************/
void VS_Write_Reg(unsigned char addr,unsigned char hdat,unsigned char ldat)
{
     while(!vs_dreq_status());//VS1003的DREQ为高电平时才接收数据
     xcs_cs(0);//打开片选，SCI有效，这样才能对功能寄存器进行读写
     SPI_Write_Byte_vs(VS_WRITE_COMMAND);  //写入操作码0x02   00000010 （功能寄存器写操作）
     SPI_Write_Byte_vs(addr);  //写入寄存器地址
     SPI_Write_Byte_vs(hdat);  //写入高字节
     SPI_Write_Byte_vs(ldat);  //写入低字节
     xcs_cs(1); //关闭片选，SCI无效
}
/******************************************************************
 - 功能描述：从VS1003的功能寄存器中读取数据（一个字）
 - 隶属模块：VS1003B模块
 - 函数属性：外部，用户可调用
 - 参数说明：addr是功能寄存器的地址
 - 返回说明：返回从VS1003的功能寄存器中读到的值 
 ******************************************************************/

unsigned int VS_Read_Reg(unsigned char addr) 
{  
    unsigned int temp=0,temp1;
    while(!vs_dreq_status());//VS1003的DREQ为高电平时才接收数据
    xcs_cs(0);//打开片选，SCI有效
    SPI_Write_Byte_vs(VS_READ_COMMAND);  //读出操作码0x03   00000011（功能寄存器读操作）
    temp =SPI_Write_Byte_vs(addr);  //写入寄存器地址 并读数据
    temp1 = temp;
    temp<<=8;
    temp|=temp1;
    xcs_cs(1); //关闭片选，SCI无效
    return temp;     //返回读到的值
}
/******************************************************************
 - 功能描述：VS1003复位及初始化（设置时钟频率及音量）
 - 隶属模块：VS1003B模块
 - 函数属性：外部，用户可调用
 - 参数说明：无
 - 返回说明：无
 ******************************************************************/
void VS_Reset()
{
    vs_reset(1);   //RES=1 
    rt_thread_delay(10);
    vs_reset(0);        //RES=0
    rt_thread_delay(10);
    vs_reset(1);    //硬件复位，XRESET低电平有效
    rt_thread_delay(10);

    VS_Write_Reg(SPI_MODE  ,0x08,0x04);  //软件复位，向0号寄存器写入0x0804   SM_SDINEW为1   SM_RESET为1
    VS_Write_Reg(SPI_CLOCKF,0x98,0x00);  //时钟设置，向3号寄存器写入0x9800   SC_MULT  为4   SC_ADD  为3   SC_FREQ为0
    VS_Write_Reg(SPI_VOL   ,0x00,0x00);  //音量设置，左右声道均最大音量
   
    xdcs_cs(0);	     //打开数据片选，注意此时XCS（命令片选）为高电平，SDI有效
    SPI_Write_Byte_vs(0);    //写入数据，这里写入4个0，是无关数据，用来启动数据传输
    SPI_Write_Byte_vs(0);
    SPI_Write_Byte_vs(0);
    SPI_Write_Byte_vs(0);
    xdcs_cs(1);	    //关闭数据片选，SDI无效
} 
/******************************************************************
 - 功能描述：向VS1003写入一个字节的音频数据（即用于播放的数据）
             注：调用前先将VS_XDCS置为0，打开数据片选
 - 隶属模块：VS1003B模块
 - 函数属性：外部，用户可调用
 - 参数说明：dat是要写入的字节
 - 返回说明：无
 ******************************************************************/

void VS_Send_Dat(unsigned char dat) 
{ 
   
   while(!vs_dreq_status());  //VS1003的DREQ为高才能写入数据
   SPI_Write_Byte_vs(dat);//通过SPI向VS1003写入一个字节的音频数据
}
/******************************************************************
 - 功能描述：向VS1003写入2048个0，用于清空VS1003的数据缓冲区
             注：在播放完一个完整的音频（如一首完整的MP3）后，调用
             此函数，清空VS1003数据缓冲区，为下面的音频数据（如下
             一首MP3）作准备。        
 - 隶属模块：VS1003B模块
 - 函数属性：外部，用户可调用
 - 参数说明：无
 - 返回说明：无
 ******************************************************************/
void VS_Flush_Buffer() 
{
    unsigned int i;
    xdcs_cs(0);	   //打开数据片选，即开启SDI传输
    for(i=0;i<2048;i++)
    {
        VS_Send_Dat(0);
    }
    xdcs_cs(1);        //关闭数据片选
}
/******************************************************************
 - 功能描述：正弦测试，这是测试VS1003芯片是否正常的有效手段！！
 - 隶属模块：VS1003B模块
 - 函数属性：外部，用户可调用
 - 参数说明：x决定了正弦测试中产生的正弦波的频率，直接影响听到的
             声音的频率      
 - 返回说明：无
 ******************************************************************/

void vs_sin_test(unsigned char x)
{ 
    vs_io_init();
    vs1003_spi_init();
    VS_Reset();
    VS_Write_Reg(0x00,0x08,0x20);//启动测试，向0号寄存器写入0x0820   SM_SDINEW为1   SM_TEST为1
    while(!vs_dreq_status());   //等待DREQ变为高电平
    xdcs_cs(0);	        //打开数据片选 SDI有效
    SPI_Write_Byte_vs(0x53);//写入以下8个字节,进入正弦测试
    SPI_Write_Byte_vs(0xef); 
    SPI_Write_Byte_vs(0x6e);
    SPI_Write_Byte_vs(x);   //参数x用来调整正弦测试中正弦波的频率   FsIdx (b7~b5):采样率表索引   S (b4~b0):正弦波的跃速   频率F=Fs X S / 128
    SPI_Write_Byte_vs(0);   //比如x=126 (0b 011 11110) FsIdx=011=3   Fs=22050Hz   S=11110=30    F=22050Hz X 30 /128 =5168 Hz
    SPI_Write_Byte_vs(0);
    SPI_Write_Byte_vs(0);
    SPI_Write_Byte_vs(0);
    rt_thread_delay(10000);      //这里延时一段时间，为了听到“正弦音”
    
    SPI_Write_Byte_vs(0x45);//写入以下8个字节，退出正弦测试
    SPI_Write_Byte_vs(0x78); 
    SPI_Write_Byte_vs(0x69);
    SPI_Write_Byte_vs(0x74);
    SPI_Write_Byte_vs(0);
    SPI_Write_Byte_vs(0);
    SPI_Write_Byte_vs(0);
    SPI_Write_Byte_vs(0);
    xdcs_cs(1);	    //关闭数据片选 ，SDI无效
} 
u8 song_buff[320];
void vs_music_test(int argc, char ** argv)
{
    char file_path[100];
    int data_len = 0;
    int ret;
    int fd;
    if (argc < 2)
    {
        rt_kprintf("Usage: err\n");
        return ;
    }
    rt_memset(file_path,0,100);
    getcwd(file_path,100);
    if (file_path[rt_strlen(file_path) - 1] != '/')
        strcat(file_path, "/");
    strcat(file_path, argv[1]);
    fd = open(file_path,O_RDONLY);
    if(fd == -1)//打开文件
	{
	    rt_kprintf("open file fail\n");
		return ;//打开文件错误
	}
	vs_io_init();
    vs1003_spi_init();
    VS_Reset();
	while(1)
	{
	    ret = read(fd,song_buff,320);//填充缓冲区
	    data_len = 0;
	    while(1)
	    {
	        int i;
	        while(!vs_dreq_status());   //等待DREQ变为高电平
	        xdcs_cs(0);
	        for(i=0;i<32;i++)
	            SPI_Write_Byte_vs(song_buff[data_len+i]);
            data_len += 32;
            xdcs_cs(1);
            if(data_len >=320)
            {
                //rt_kprintf(" data over \n");
                break;
            }
	    }
	}
    
}
#ifdef RT_USING_FINSH
#include <finsh.h>
FINSH_FUNCTION_EXPORT(vs_sin_test, vs_sin_test);
MSH_CMD_EXPORT(vs_music_test, vs_music_test);

#endif

/******************************************************************
 - 功能描述：为VS1003打补丁，获得实时频谱
             注：atab与dtab是VS1003频谱功能补丁码，在patch.h中
 - 隶属模块：VS1003B模块
 - 函数属性：外部，用户可调用
 - 参数说明：无    
 - 返回说明：无
 ******************************************************************/

/*void LoadPatch() 
{
 unsigned int i;
 for(i=0;i<943;i++)
 {
  VS_Write_Reg(atab[i],dtab[i]>>8,dtab[i]&0xff);
 }
}*/
