//文件名：vs1003.c
//功能：vs1003 的底层驱动程序，主控芯片为stm32f4；
//     其他的微处理器(带SPI接口的)只需稍加修改即可适用；
#include <rtthread.h>
#include <rtdevice.h>
#include "vs1003.h"
#include <string.h>
#include <dfs.h>
#include <dfs_posix.h>

#define vs1003_printf rt_kprintf
#define MUSIC_DEBUG
#ifdef MUSIC_DEBUG
#define music_log(fmt,arg...) rt_kprintf("MUSIC->%s - L:%d  "fmt"\n",__FUNCTION__, __LINE__, ##arg);
#else
#define music_log
#endif

#define RT_MUSIC_THREAD_PRIORITY 5
#define VS_1003_BUFF_LEN 4*1024
#define VS_FLIE_NAME_LEN 128
typedef struct 
{
    char file_name[50];
}MESSAGE;
typedef struct 
{
    int fd;//文件句柄
    
    int play_way;//循环播放或单曲
    int file_len;
    char file_name[VS_FLIE_NAME_LEN];
}VS_CTL;

static u8 vs1003_data_buff[VS_1003_BUFF_LEN];
static u8 spi_dma_status = 0;

#define VS_START_EVENT (1<<0)//开始事件
#define VS_STOP_EVENT  (1<<1)//停止事件
#define VS_DMA_TC_EVENT (1<<2)//DMA完成事件
#define VS_DMA_HT_EVENT (1<<3)//DMA半完成事件
#define VS_PLAY_OVER_EVENT (1<<4)//DMA半完成事件

#define VS_PLAY_ONE 0
#define VS_PLAY_CIRCLE 1

static rt_timer_t vs_timer;
static struct rt_event vs_event;
VS_CTL vs_ctl;

u8 vs_dma_status(void)
{
    return spi_dma_status;
}
void vs_spi_dma_init(void)
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
	
	DMA_ClearITPendingBit(DMA1_Stream4,DMA_IT_TCIF4); //传输完成中断悬挂位
	DMA_ClearITPendingBit(DMA1_Stream4,DMA_IT_HTIF4);//传输一半完成中断悬挂位
	DMA_ITConfig(DMA1_Stream4,DMA_IT_TC,ENABLE);//打开全部传输完整中断也就是4K数据
	DMA_ITConfig(DMA1_Stream4,DMA_IT_HT,ENABLE);//打开传输完一半数据中断，也就是传输完2K
	
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream4_IRQn;//设置DMA中断
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);
	DMA_Cmd(DMA1_Stream4, ENABLE);//打开该SPI的控制器流
	SPI_I2S_DMACmd(SPI2,SPI_I2S_DMAReq_Tx,DISABLE);
}

void vs_spi_dma_stop()
{
    SPI_I2S_DMACmd(SPI2,SPI_I2S_DMAReq_Tx,DISABLE);
    vs_xdcs_cs(1);
    spi_dma_status = 0;
}
void vs_spi_dma_start()
{
    vs_xdcs_cs(0);
    SPI_I2S_DMACmd(SPI2,SPI_I2S_DMAReq_Tx,ENABLE);
    spi_dma_status = 1;
}
void vs_spi_stop()
{
    SPI_Cmd(SPI2, DISABLE);
    vs_xdcs_cs(1);
}
void vs_spi_start()
{
    vs_xdcs_cs(0);
    SPI_Cmd(SPI2, ENABLE);
}

void vs_dreq_int(void)//vs下降沿中断
{
    vs1003_printf("c6\n");
    vs_spi_dma_stop();
}
void vs_spi_init(void)
{ 
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
    SPI_CalculateCRC(SPI2, DISABLE);
     /* Enable SPI_MASTER */
    SPI_Cmd(SPI2, ENABLE);
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
void vs_reset_cmd(u8 status)
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
void vs_xcs_cs(u8 status)
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
void vs_xdcs_cs(u8 status)
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
     vs_xcs_cs(0);//打开片选，SCI有效，这样才能对功能寄存器进行读写
     SPI_Write_Byte_vs(VS_WRITE_COMMAND);  //写入操作码0x02   00000010 （功能寄存器写操作）
     SPI_Write_Byte_vs(addr);  //写入寄存器地址
     SPI_Write_Byte_vs(hdat);  //写入高字节
     SPI_Write_Byte_vs(ldat);  //写入低字节
     vs_xcs_cs(1); //关闭片选，SCI无效
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
    vs_xcs_cs(0);//打开片选，SCI有效
    SPI_Write_Byte_vs(VS_READ_COMMAND);  //读出操作码0x03   00000011（功能寄存器读操作）
    temp =SPI_Write_Byte_vs(addr);  //写入寄存器地址 并读数据
    temp1 = temp;
    temp<<=8;
    temp|=temp1;
    vs_xcs_cs(1); //关闭片选，SCI无效
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
    vs_reset_cmd(1);   //RES=1 
    rt_thread_delay(10);
    vs_reset_cmd(0);        //RES=0
    rt_thread_delay(10);
    vs_reset_cmd(1);    //硬件复位，XRESET低电平有效
    rt_thread_delay(10);

    VS_Write_Reg(SPI_MODE  ,0x08,0x04);  //软件复位，向0号寄存器写入0x0804   SM_SDINEW为1   SM_RESET为1
    VS_Write_Reg(SPI_CLOCKF,0x98,0x00);  //时钟设置，向3号寄存器写入0x9800   SC_MULT  为4   SC_ADD  为3   SC_FREQ为0
    VS_Write_Reg(SPI_VOL   ,0x00,0x00);  //音量设置，左右声道均最大音量
   
    vs_xdcs_cs(0);	     //打开数据片选，注意此时XCS（命令片选）为高电平，SDI有效
    SPI_Write_Byte_vs(0);    //写入数据，这里写入4个0，是无关数据，用来启动数据传输
    SPI_Write_Byte_vs(0);
    SPI_Write_Byte_vs(0);
    SPI_Write_Byte_vs(0);
    vs_xdcs_cs(1);	    //关闭数据片选，SDI无效
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
    vs_xdcs_cs(0);	   //打开数据片选，即开启SDI传输
    for(i=0;i<2048;i++)
    {
        VS_Send_Dat(0);
    }
    vs_xdcs_cs(1);        //关闭数据片选
}

void vs1003_init(void)
{
    vs_io_init();
    vs_spi_init();
    VS_Reset();
    return ;
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
    vs_spi_init();
    VS_Reset();
    VS_Write_Reg(0x00,0x08,0x20);//启动测试，向0号寄存器写入0x0820   SM_SDINEW为1   SM_TEST为1
    while(!vs_dreq_status());   //等待DREQ变为高电平
    vs_xdcs_cs(0);	        //打开数据片选 SDI有效
    SPI_Write_Byte_vs(0x53);//写入以下8个字节,进入正弦测试
    SPI_Write_Byte_vs(0xef); 
    SPI_Write_Byte_vs(0x6e);
    SPI_Write_Byte_vs(x);   //参数x用来调整正弦测试中正弦波的频率   FsIdx (b7~b5):采样率表索引   S (b4~b0):正弦波的跃速   频率F=Fs X S / 128
    SPI_Write_Byte_vs(0);   //比如x=126 (0b 011 11110) FsIdx=011=3   Fs=22050Hz   S=11110=30    F=22050Hz X 30 /128 =5168 Hz
    SPI_Write_Byte_vs(0);
    SPI_Write_Byte_vs(0);
    SPI_Write_Byte_vs(0);
    rt_thread_delay(4000);      //这里延时一段时间，为了听到“正弦音”
    
    SPI_Write_Byte_vs(0x45);//写入以下8个字节，退出正弦测试
    SPI_Write_Byte_vs(0x78); 
    SPI_Write_Byte_vs(0x69);
    SPI_Write_Byte_vs(0x74);
    SPI_Write_Byte_vs(0);
    SPI_Write_Byte_vs(0);
    SPI_Write_Byte_vs(0);
    SPI_Write_Byte_vs(0);
    vs_xdcs_cs(1);	    //关闭数据片选 ，SDI无效
}

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
    vs_spi_init();
    VS_Reset();
    read(fd,vs1003_data_buff,4096);
    vs_xdcs_cs(0);
    vs_spi_dma_init();
    vs_spi_dma_start();
    rt_kprintf("rt_timer_create ok\n");
	while(0)
	{
	    ret = read(fd,vs1003_data_buff,320);//填充缓冲区
	    data_len = 0;
	    while(1)
	    {
	        int i;
	        while(!vs_dreq_status());   //等待DREQ变为高电平
	        vs_xdcs_cs(0);
	        for(i=0;i<32;i++)
	            SPI_Write_Byte_vs(vs1003_data_buff[data_len+i]);
            data_len += 32;
            vs_xdcs_cs(1);
            if(data_len >=320)
            {
                //rt_kprintf(" data over \n");
                break;
            }
	    }
	}
    
}
void DMA1_Stream4_IRQHandler(void)
{
    
	if(DMA_GetITStatus(DMA1_Stream4,DMA_IT_TCIF4))//传输完成中断标志
    {
        rt_kprintf("tc\n");
        rt_event_send(&vs_event,VS_DMA_TC_EVENT);
    }
	else if(DMA_GetITStatus(DMA1_Stream4,DMA_IT_HTIF4))//传输一半完成中断标志
	{
	    rt_kprintf("ht\n");
        rt_event_send(&vs_event,VS_DMA_HT_EVENT);
    }
	DMA_ClearITPendingBit(DMA1_Stream4,DMA_IT_TCIF4 | DMA_IT_HTIF4);
}

static void vs_music_service_task(void *param)
{
    rt_err_t ret;
    rt_uint32_t recved;
    rt_memset(&vs_ctl,0,sizeof(VS_CTL));
    rt_memset(&vs_event,0,sizeof(rt_event));
    ret = rt_event_init(&vs_event, "vs_event", RT_IPC_FLAG_FIFO);
    if(ret!=RT_EOK)
    {
        rt_kprintf("rt_event_init fail\n");
        return ;
    }
    vs1003_init();
    while (1)
    {
        
        if(rt_event_recv(&vs_event,VS_START_EVENT | VS_STOP_EVENT,VS_DMA_TC_EVENT | VS_DMA_HT_EVENT |VS_PLAY_OVER_EVENT,
                            RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                            rt_tick_from_millisecond(10),&recved)!= RT_EOK);
        {
            if(vs_dreq_status())//定时检测数据请求引脚，打开SPI DMA
            {
                vs_spi_dma_start();//DMA会在下降沿触发时关掉
            }
            continue;
        }
        //获得事件
        if(recved & VS_START_EVENT)
        {
            rt_kprintf("recvied start cmd\n");
            vs_spi_dma_start();
        }
        if(recved & VS_STOP_EVENT)
        {
            rt_kprintf("recvied stop cmd\n");
            vs_spi_dma_stop();
        }
        if(recved & VS_PLAY_OVER_EVENT)
        {
            rt_kprintf("play over\n");
            if(vs_ctl.play_way == VS_PLAY_CIRCLE)
            {
                vs_auto_play("/music/");
            }
        }
        if(recved & VS_DMA_TC_EVENT)
        {
            int ret;
            
            if(vs_ctl.fd > 0)
            {
                ret = read(vs_ctl.fd,vs1003_data_buff+(VS_1003_BUFF_LEN>>1),(VS_1003_BUFF_LEN>>1));
                rt_kprintf("DMA all ok len:%d\n",ret);
            }
            
        }
        if(recved & VS_DMA_HT_EVENT)
        {
            int ret;
            if(vs_ctl.fd > 0)
            {
                ret = read(vs_ctl.fd,vs1003_data_buff,(VS_1003_BUFF_LEN>>1));
                rt_kprintf("DMA half ok len:%d\n"ret);
            }
        }
    }
}

int vs_auto_play(char * file_dir_name)
{
    char temp_file_name[60];
    static  DIR * dir = NULL;
    struct dirent * file_info = NULL;
    if(!dir)
    {
        dir = opendir(file_dir_name);
        if(!dir)
        {
            rt_kprintf("%s is empty\n",file_dir_name);
            return -1;
        }
    }
    file_info = readdir(dir);
    if(!file_info)
    {
        seekdir(dir,SEEK_SET);
        file_info = readdir(dir);
        return -1;
    }
    rt_memset(temp_file_name,0,60);
    rt_strncpy(temp_file_name,file_dir_name,rt_strlen(file_dir_name));
    rt_strncpy(temp_file_name+rt_strlen(temp_file_name),file_info->d_name,rt_strlen(file_info->d_name));
    rt_kprintf("当前文件为=%s\n",temp_file_name);
    dev_audio_play(temp_file_name);
    return 0;
}

void vs_music_startup(void)
{
    rt_thread_t tid;
    tid = rt_thread_find("vs1003");
    if(tid)
    {
        rt_kprintf("runing this task\n");
        return;
    }
    tid = rt_thread_create("vs1003",
                           vs_music_service_task, 
                           (void *) 0,
                           2048*4,
                           RT_MUSIC_THREAD_PRIORITY,
                           20);
    if (tid != RT_NULL)
    {
        rt_thread_startup(tid);
        music_log("music_service_task init done...\n");
    }
    else
    {
        music_log("music_service_task init fail...\n");
    }
}
int vs_music_action(int argc, char ** argv)
{
    rt_thread_t tid;
    if (argc < 2)
    {
        music_log("Usage: err\n");
        return -RT_ERROR;
    }
    tid = rt_thread_find("vs1003");
    if(!tid)
    {
        rt_kprintf("please run this task\n");
        return -1;
    }
    if(!strncmp(argv[1],"song",rt_strlen("song")))
    {
        rt_err_t err;
        if(argc < 3)
        {
            music_log("please input the name of songs\n");
            return -RT_ERROR;
        }
        if(err == RT_EOK)
        {
            //music_log("rt_mq_send done!\n");
        }
        else
        {
            //music_log("rt_mq_send fail!\n");
            return -RT_ERROR;
        }
        return RT_EOK;
    }
    if(!strncmp(argv[1],"stop",rt_strlen("stop")))
    {
        rt_err_t err;
        return RT_EOK;
    }
 return 0;
}
void vs_1003_cmd(int argc, char ** argv)
{
    if (argc != 2)
    {
        music_log("Usage: err\n");
        return;
    }
    if(!strncmp(argv[1],"run",rt_strlen("run")))
    {
        
    }
    if(!strncmp(argv[1],"stop",rt_strlen("st")))
    {
        
    }
}
//INIT_APP_EXPORT(music_startup);

int dev_audio_play(char* file_name)
{
    int fd;
    int ret;
    rt_thread_t tid;
    char temp_file[128];
    if(file_name[0] != '/')
	{
	    rt_memset(temp_file,0,128);
	    getcwd(temp_file,128);
	    if (temp_file[rt_strlen(temp_file) - 1]  != '/')
            strcat(temp_file, "/");
	}
    strcat(temp_file, file_name);
    fd = open(temp_file,O_RDONLY);
    if(fd < 0)
    {
        rt_kprintf("open %s fail\n",temp_file);
        return -1;
    }
    vs_spi_dma_stop();
    if(vs_ctl.fd >= 0)
    {
        close(vs_ctl.fd);
    }
    rt_memset(&vs_ctl,0,sizeof(VS_CTL));
    vs_ctl.fd = fd;
    rt_memcpy(vs_ctl.file_name,temp_file,rt_strlen(temp_file));

    tid = rt_thread_find("vs1003");
    if(!tid)
    {
        rt_kprintf("dev_audio_play run on vs1003\n");
        vs_music_startup();//启动vs1003服务例程
    }
    rt_memset(vs1003_data_buff,0,VS_1003_BUFF_LEN);
    ret = read(vs_ctl.fd,vs1003_data_buff,VS_1003_BUFF_LEN);
    vs_spi_dma_init();
    vs_spi_dma_start();
    
    return RT_EOK;
}
int dev_audio_open(void)
{
    return RT_EOK;
}
int dev_audio_close(void)
{
    return RT_EOK;
}
int dev_audio_stop(void)
{
    return RT_EOK;
}
int dev_audio_pause(void)
{
    
    return RT_EOK;
}
int dev_audio_ioctl(int nCmd ,int lParam ,int wParam)
{
    
    return RT_EOK;
}

#ifdef RT_USING_FINSH
#include <finsh.h>
MSH_CMD_EXPORT_ALIAS(vs_music_startup,vs, vs_music_startup vs1003任务);
MSH_CMD_EXPORT_ALIAS(vs_music_action,  vscmd,  vscmd);
MSH_CMD_EXPORT_ALIAS(vs_1003_cmd, vsspi, vsspi);
FINSH_FUNCTION_EXPORT(vs_sin_test, vs_sin_test);
MSH_CMD_EXPORT(vs_music_test, vs_music_test);
#endif

