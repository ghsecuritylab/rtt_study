//�ļ�����vs1003.c
//���ܣ�vs1003 �ĵײ�������������оƬΪstm32f4��
//     ������΢������(��SPI�ӿڵ�)ֻ���Լ��޸ļ������ã�
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

#if 0 /*xqy 2019-12-26*/
#define CS_DEVICE_NAME "spi20"
#define DCS_DEVICE_NAME "spi21"
struct rt_spi_device *vs1003_cs_device = NULL;//�����豸
struct rt_spi_device *vs1003_dcs_device = NULL;//�����豸
//��������������ܣ����Խ������ֱ�Ӳ���spi,��Ϊ��������Ƚ�Ƶ���Ļ���
//�����豸�ڴ����ʱ��Ҫ��������spi����Ȼ���ǵ����ö�����ͬ��
#endif
#define VS_1003_BUFF_LEN 4*1024
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
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);//������ʱ��DMAspi2 tx��4

    //SPI2 TX DMA����DMA1 ��4,0ͨ�� 
    DMA_DeInit(DMA1_Stream4);//���֮ǰ��stream4�ϵ������жϱ�־
    while (DMA_GetCmdStatus(DMA1_Stream4) != DISABLE){}//�ȴ�DMA������ 
    
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;  //ͨ��ѡ��
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&(SPI2->DR);
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)vs1003_data_buff;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;//�ڴ浽����
	DMA_InitStructure.DMA_BufferSize = VS_1003_BUFF_LEN;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//�����ַ�Զ���������
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//�ڴ��Զ����Ӵ�
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//����ģʽ��ȡ��4K���ݺ��ٴ���ʼλ�ÿ�ʼȡ��ѭ������ֹ
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//�洢��ͻ�����δ���
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//����ͻ�����δ���
	DMA_Init(DMA1_Stream4, &DMA_InitStructure);
	
	DMA_ClearITPendingBit(DMA1_Stream4,DMA_IT_TCIF5); //��������ж�����λ
	DMA_ClearITPendingBit(DMA1_Stream4,DMA_IT_HTIF5);//����һ������ж�����λ
	DMA_ITConfig(DMA1_Stream4,DMA_IT_TC,ENABLE);//��ȫ�����������ж�Ҳ����4K����
	DMA_ITConfig(DMA1_Stream4,DMA_IT_HT,ENABLE);//�򿪴�����һ�������жϣ�Ҳ���Ǵ�����2K
	
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream4_IRQn;//����DMA�ж�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);
	DMA_Cmd(DMA1_Stream4, ENABLE);//�򿪸�SPI�Ŀ�������
	vs_spi_dma_stop();
}

void vs_spi_dma_stop()
{
    SPI_Cmd(SPI2, DISABLE);
    SPI_I2S_DMACmd(SPI2,SPI_I2S_DMAReq_Tx,DISABLE);
    xdcs_cs(1);
    spi_dma_status = 0;
}
void vs_spi_dma_start()
{
    xdcs_cs(0);
    SPI_Cmd(SPI2, ENABLE);
    SPI_I2S_DMACmd(SPI2,SPI_I2S_DMAReq_Tx,ENABLE);
    spi_dma_status = 1;
}

void pc6_gpio_int_handler(void)
{
    vs1003_printf("c6\n");
    vs_spi_dma_stop();
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

    rt_memset(vs1003_data_buff,0,VS_1003_BUFF_LEN);//���BUFF������
    /* Enable GPIO clock */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;//
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
    GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_2;
    GPIO_Init(GPIOC, &GPIO_InitStructure);//��ʼ��
    
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_SPI2); 
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource2,GPIO_AF_SPI2); 
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource3,GPIO_AF_SPI2); 
 
	//����ֻ���SPI�ڳ�ʼ��
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2,ENABLE);//��λSPI2
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2,DISABLE);//ֹͣ��λSPI2

    SPI_StructInit(&SPI_InitStructure);
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    /* baudrate *///��������Ƶ��Ϊ84MHZ��32��Ƶ�����ٳ���32���Եõ�SPI��Ƶ
    //�����͵�������vs1003���ղ�����OK������Ƶ��
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    /* CPHA */
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    /* MSB or LSB */
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;//ȫ˫��
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_NSS  = SPI_NSS_Soft;//cs�����
    
    /* init SPI */
    SPI_I2S_DeInit(SPI2);
    SPI_Init(SPI2, &SPI_InitStructure);
    /* Enable SPI_MASTER */
    SPI_Cmd(SPI2, ENABLE);
    SPI_CalculateCRC(SPI2, DISABLE);
}
/**********************************************************/
/*  �������� :  InitPortVS1003                            */
/*  �������� �� MCU��vs1003�ӿڵĳ�ʼ��                   */
/*  ����     :  ��                                        */
/*  ����ֵ   :  ��                                        */
/*--------------------------------------------------------*/
/*stm32f4��������vs1003�Ľӿڶ���
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
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//ʹ��SYSCFGʱ��
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;	//VS_XRESET
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
    GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;//xcs�������Ƭѡ
    GPIO_Init(GPIOC, &GPIO_InitStructure);//��ʼ��
    GPIO_SetBits(GPIOC, GPIO_Pin_13);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;//xdcs���ݿ���Ƭѡ
    GPIO_Init(GPIOC, &GPIO_InitStructure);//��ʼ��
    GPIO_SetBits(GPIOC, GPIO_Pin_0);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;//VS_DREQ
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_Init(GPIOC, &GPIO_InitStructure);//��ʼ��

    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC,GPIO_PinSource6);
    /* ����EXTI_Line11 */
    EXTI_InitStructure.EXTI_Line = EXTI_Line6;//LINE11
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//�ж��¼�
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //�½��ش��� 
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;//ʹ��LINE0
    EXTI_Init(&EXTI_InitStructure);//����

	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;//�ⲿ�ж�0
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//��ռ���ȼ�0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;//�����ȼ�2
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//ʹ���ⲿ�ж�ͨ��
    NVIC_Init(&NVIC_InitStructure);//����
}
s32 vs_dreq_status(void)//vs1003�Ƿ��пռ�������ݣ��ߵ�ƽ:�пռ䣬�͵�ƽ:�޿ռ�
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
/*  �������� :  SPIPutChar                                */
/*  �������� �� ͨ��SPI����һ���ֽڵ�����                 */
/*  ����     :  �����͵��ֽ�����                          */
/*  ����ֵ   :  ��                                        */
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
 - ������������VS1003�Ĺ��ܼĴ�����д�����ݣ�һ���֣��������ֽڣ�
 - ����ģ�飺VS1003Bģ��
 - �������ԣ��ⲿ���û��ɵ���
 - ����˵����addr�ǹ��ܼĴ����ĵ�ַ
             hdat��Ҫд��ĸ��ֽ�
             ldat��Ҫд��ĵ��ֽ�
 - ����˵�����޷���
 ******************************************************************/
void VS_Write_Reg(unsigned char addr,unsigned char hdat,unsigned char ldat)
{
     while(!vs_dreq_status());//VS1003��DREQΪ�ߵ�ƽʱ�Ž�������
     xcs_cs(0);//��Ƭѡ��SCI��Ч���������ܶԹ��ܼĴ������ж�д
     SPI_Write_Byte_vs(VS_WRITE_COMMAND);  //д�������0x02   00000010 �����ܼĴ���д������
     SPI_Write_Byte_vs(addr);  //д��Ĵ�����ַ
     SPI_Write_Byte_vs(hdat);  //д����ֽ�
     SPI_Write_Byte_vs(ldat);  //д����ֽ�
     xcs_cs(1); //�ر�Ƭѡ��SCI��Ч
}
/******************************************************************
 - ������������VS1003�Ĺ��ܼĴ����ж�ȡ���ݣ�һ���֣�
 - ����ģ�飺VS1003Bģ��
 - �������ԣ��ⲿ���û��ɵ���
 - ����˵����addr�ǹ��ܼĴ����ĵ�ַ
 - ����˵�������ش�VS1003�Ĺ��ܼĴ����ж�����ֵ 
 ******************************************************************/

unsigned int VS_Read_Reg(unsigned char addr) 
{  
    unsigned int temp=0,temp1;
    while(!vs_dreq_status());//VS1003��DREQΪ�ߵ�ƽʱ�Ž�������
    xcs_cs(0);//��Ƭѡ��SCI��Ч
    SPI_Write_Byte_vs(VS_READ_COMMAND);  //����������0x03   00000011�����ܼĴ�����������
    temp =SPI_Write_Byte_vs(addr);  //д��Ĵ�����ַ ��������
    temp1 = temp;
    temp<<=8;
    temp|=temp1;
    xcs_cs(1); //�ر�Ƭѡ��SCI��Ч
    return temp;     //���ض�����ֵ
}
/******************************************************************
 - ����������VS1003��λ����ʼ��������ʱ��Ƶ�ʼ�������
 - ����ģ�飺VS1003Bģ��
 - �������ԣ��ⲿ���û��ɵ���
 - ����˵������
 - ����˵������
 ******************************************************************/
void VS_Reset()
{
    vs_reset(1);   //RES=1 
    rt_thread_delay(10);
    vs_reset(0);        //RES=0
    rt_thread_delay(10);
    vs_reset(1);    //Ӳ����λ��XRESET�͵�ƽ��Ч
    rt_thread_delay(10);

    VS_Write_Reg(SPI_MODE  ,0x08,0x04);  //�����λ����0�żĴ���д��0x0804   SM_SDINEWΪ1   SM_RESETΪ1
    VS_Write_Reg(SPI_CLOCKF,0x98,0x00);  //ʱ�����ã���3�żĴ���д��0x9800   SC_MULT  Ϊ4   SC_ADD  Ϊ3   SC_FREQΪ0
    VS_Write_Reg(SPI_VOL   ,0x00,0x00);  //�������ã������������������
   
    xdcs_cs(0);	     //������Ƭѡ��ע���ʱXCS������Ƭѡ��Ϊ�ߵ�ƽ��SDI��Ч
    SPI_Write_Byte_vs(0);    //д�����ݣ�����д��4��0�����޹����ݣ������������ݴ���
    SPI_Write_Byte_vs(0);
    SPI_Write_Byte_vs(0);
    SPI_Write_Byte_vs(0);
    xdcs_cs(1);	    //�ر�����Ƭѡ��SDI��Ч
} 
/******************************************************************
 - ������������VS1003д��һ���ֽڵ���Ƶ���ݣ������ڲ��ŵ����ݣ�
             ע������ǰ�Ƚ�VS_XDCS��Ϊ0��������Ƭѡ
 - ����ģ�飺VS1003Bģ��
 - �������ԣ��ⲿ���û��ɵ���
 - ����˵����dat��Ҫд����ֽ�
 - ����˵������
 ******************************************************************/

void VS_Send_Dat(unsigned char dat) 
{ 
   
   while(!vs_dreq_status());  //VS1003��DREQΪ�߲���д������
   SPI_Write_Byte_vs(dat);//ͨ��SPI��VS1003д��һ���ֽڵ���Ƶ����
}
/******************************************************************
 - ������������VS1003д��2048��0���������VS1003�����ݻ�����
             ע���ڲ�����һ����������Ƶ����һ��������MP3���󣬵���
             �˺��������VS1003���ݻ�������Ϊ�������Ƶ���ݣ�����
             һ��MP3����׼����        
 - ����ģ�飺VS1003Bģ��
 - �������ԣ��ⲿ���û��ɵ���
 - ����˵������
 - ����˵������
 ******************************************************************/
void VS_Flush_Buffer() 
{
    unsigned int i;
    xdcs_cs(0);	   //������Ƭѡ��������SDI����
    for(i=0;i<2048;i++)
    {
        VS_Send_Dat(0);
    }
    xdcs_cs(1);        //�ر�����Ƭѡ
}

void vs1003_init(void)
{
    vs_io_init();
    vs1003_spi_init();
    VS_Reset();
    return ;
}
/******************************************************************
 - �������������Ҳ��ԣ����ǲ���VS1003оƬ�Ƿ���������Ч�ֶΣ���
 - ����ģ�飺VS1003Bģ��
 - �������ԣ��ⲿ���û��ɵ���
 - ����˵����x���������Ҳ����в��������Ҳ���Ƶ�ʣ�ֱ��Ӱ��������
             ������Ƶ��      
 - ����˵������
 ******************************************************************/

void vs_sin_test(unsigned char x)
{ 
    vs_io_init();
    vs1003_spi_init();
    VS_Reset();
    VS_Write_Reg(0x00,0x08,0x20);//�������ԣ���0�żĴ���д��0x0820   SM_SDINEWΪ1   SM_TESTΪ1
    while(!vs_dreq_status());   //�ȴ�DREQ��Ϊ�ߵ�ƽ
    xdcs_cs(0);	        //������Ƭѡ SDI��Ч
    SPI_Write_Byte_vs(0x53);//д������8���ֽ�,�������Ҳ���
    SPI_Write_Byte_vs(0xef); 
    SPI_Write_Byte_vs(0x6e);
    SPI_Write_Byte_vs(x);   //����x�����������Ҳ��������Ҳ���Ƶ��   FsIdx (b7~b5):�����ʱ�����   S (b4~b0):���Ҳ���Ծ��   Ƶ��F=Fs X S / 128
    SPI_Write_Byte_vs(0);   //����x=126 (0b 011 11110) FsIdx=011=3   Fs=22050Hz   S=11110=30    F=22050Hz X 30 /128 =5168 Hz
    SPI_Write_Byte_vs(0);
    SPI_Write_Byte_vs(0);
    SPI_Write_Byte_vs(0);
    rt_thread_delay(10000);      //������ʱһ��ʱ�䣬Ϊ����������������
    
    SPI_Write_Byte_vs(0x45);//д������8���ֽڣ��˳����Ҳ���
    SPI_Write_Byte_vs(0x78); 
    SPI_Write_Byte_vs(0x69);
    SPI_Write_Byte_vs(0x74);
    SPI_Write_Byte_vs(0);
    SPI_Write_Byte_vs(0);
    SPI_Write_Byte_vs(0);
    SPI_Write_Byte_vs(0);
    xdcs_cs(1);	    //�ر�����Ƭѡ ��SDI��Ч
}
int temp_fd = 0;
static rt_timer_t timer1;

void DMA1_Stream4_IRQHandler(void)
{
    //rt_interrupt_enter();
    //rt_sem_release(dma_int);
    //DataRequestFlag = 1;
    
	if(DMA_GetITStatus(DMA1_Stream4,DMA_IT_TCIF4))//��������жϱ�־
    {
       // DataRequestFlag = 1;
       rt_kprintf("tc\n");
       if(temp_fd)
       {
            u32 ret;
            ret = read(temp_fd,vs1003_data_buff+2048,2048);
            if(ret!=2048)
            {
                rt_kprintf("data over\n");
                vs_spi_dma_stop();
            }
       }
            
    }
	else if(DMA_GetITStatus(DMA1_Stream5,DMA_IT_HTIF5))//����һ������жϱ�־
	{
	    rt_kprintf("ht\n");
       // DataRequestFlag = 2;
       if(temp_fd)
       {
            u32 ret;
            ret = read(temp_fd,vs1003_data_buff,2048);
            if(ret!=2048)
            {
                rt_kprintf("data over\n");
                vs_spi_dma_stop();
            }
       }
    }
	DMA_ClearITPendingBit(DMA1_Stream5,DMA_IT_TCIF5 | DMA_IT_HTIF5);
	//rt_interrupt_leave();
}

u8 song_buff[320];
void time_out_deal(void)
{
    rt_kprintf("r\n");
    if(vs_dreq_status())
    {
        vs_spi_dma_start();
    }
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
    if(fd == -1)//���ļ�
	{
	    rt_kprintf("open file fail\n");
		return ;//���ļ�����
	}
	timer1 = rt_timer_create("vs_timer_app",
                             time_out_deal,
                             RT_NULL,
                             10,
                             RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    if (timer1 != RT_NULL)
    {
        rt_timer_start(timer1);
    }
    else
    {
        rt_kprintf("rt_timer_create fail\n");
        return ;
    }
	vs_io_init();
    vs1003_spi_init();
    VS_Reset();
    temp_fd = fd;
    read(fd,vs1003_data_buff,4096);
    xdcs_cs(0);
    vs1003_spi_dma_init();
    vs_spi_dma_start();
    rt_kprintf("rt_timer_create ok\n");
	while(0)
	{
	    ret = read(fd,song_buff,320);//��仺����
	    data_len = 0;
	    while(1)
	    {
	        int i;
	        while(!vs_dreq_status());   //�ȴ�DREQ��Ϊ�ߵ�ƽ
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

typedef struct 
{
    char file_name[50];
}MESSAGE;
typedef struct 
{
    int fd;
    int file_len;
    char file_name[30];
}VS_CTL;
#define RT_MUSIC_THREAD_PRIORITY 5


static rt_mq_t vs_music_mq = RT_NULL;
rt_sem_t vs_dma_int = RT_NULL;
rt_sem_t vs_music_break = RT_NULL;
static MESSAGE vs_mb_message;
VS_CTL vs_ctl;
static void vs_music_service_task(void *param)
{
    char file_path[256];
    rt_memset(&vs_mb_message,0,sizeof(MESSAGE));
    vs_music_mq = rt_mq_create("vs_music_mq",100,1,RT_IPC_FLAG_FIFO);
    if (vs_music_mq == RT_NULL)
    {
        music_log("music_mb create fail!\n");
        return ;
    }
    vs_dma_int = rt_sem_create("vs_dma_int",0,RT_IPC_FLAG_FIFO);
    if (vs_dma_int == RT_NULL)
    {
        music_log("dma_int create fail!\n");
        return ;
    }
    vs_music_break = rt_sem_create("vs_music_break",0,RT_IPC_FLAG_FIFO);
    if (vs_music_break == RT_NULL)
    {
        music_log("music_break create fail!\n");
        return ;
    }
    vs1003_init();
    while (1)
    {
        rt_memset(file_path,0,256);
        if (rt_mq_recv(vs_music_mq, (void*)&vs_mb_message, sizeof(MESSAGE),RT_WAITING_FOREVER) != RT_EOK)
        {
            int ret;
            ret = vs_auto_play("/music/",file_path);
            if(ret)
                continue;
        }
        rt_sem_control(vs_music_break,RT_IPC_CMD_RESET,0);//���ź�������
        AudioPlayFile(rt_strlen(file_path) >0 ? file_path : vs_mb_message.file_name);
    }
}
int vs_auto_play(char * file_dir_name,char * path)
{
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
    rt_strncpy(path,file_dir_name,rt_strlen(file_dir_name));
    rt_strncpy(path+rt_strlen(path),file_info->d_name,rt_strlen(file_info->d_name));
    rt_kprintf("��ǰ�ļ�Ϊ=%s\n",path);
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
                           2048,
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
    MESSAGE message_oo;
    if(!vs_music_mq)
    {
        music_log("please init music service task first\n");
        return -RT_ERROR;
    }
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
        rt_sem_release(vs_dma_int);
        rt_sem_release(vs_music_break);
        rt_memset(&message_oo,0,sizeof(MESSAGE));
        rt_strncpy(message_oo.file_name,argv[2],rt_strlen(argv[2]));
        err = rt_mq_send(vs_music_mq,(void*)&message_oo,sizeof(MESSAGE));
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
        rt_sem_release(vs_dma_int);
        rt_sem_release(vs_music_break);
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
        Play_Start();
    }
    if(!strncmp(argv[1],"stop",rt_strlen("st")))
    {
        Play_Stop();
    }
}
//INIT_APP_EXPORT(music_startup);

#ifdef RT_USING_FINSH
#include <finsh.h>
MSH_CMD_EXPORT_ALIAS(vs_music_startup,vs, vs_music_startup vs1003����);
MSH_CMD_EXPORT_ALIAS(vs_music_action,  vscmd,  vscmd);
MSH_CMD_EXPORT_ALIAS(vs_1003_cmd, vsspi, vsspi);

#endif

