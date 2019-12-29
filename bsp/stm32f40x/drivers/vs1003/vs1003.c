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

#define RT_VS1003_THREAD_PRIORITY 5
#define VS_1003_BUFF_LEN 1//4*1024
#define VS_FLIE_NAME_LEN 128
#define VS_FLIE_BUFFER_LEN 512

typedef struct 
{
    char file_name[50];
}MESSAGE;
typedef struct 
{
    int fd;//�ļ����
    int vs_status;//0��ʾδ�򿪣�1��ʾ���Ÿ�����2��ʾ¼��
    int play_way;//ѭ�����Ż���
    int file_len;
    int data_len;//buffer�е����ݳ���
    char file_name[VS_FLIE_NAME_LEN];
    char data_buffer[VS_FLIE_BUFFER_LEN];
}VS_CTL;

static u8 vs1003_data_buff[VS_1003_BUFF_LEN];
static u8 spi_dma_status = 0;

#define VS_START_EVENT (1<<0)//��ʼ�¼�
#define VS_STOP_EVENT  (1<<1)//ֹͣ�¼�
#define VS_DMA_TC_EVENT (1<<2)//DMA����¼�
#define VS_DMA_HT_EVENT (1<<3)//DMA������¼�
#define VS_PLAY_OVER_EVENT (1<<4)//DMA������¼�
#define VS_DATA_OVER_EVENT (1<<4)//����Ϊ���¼�

#define VS_PLAY_SHOOT 0
#define VS_PLAY_CIRCLE 1
#define VS_PLAY_SINGER_CIRCLE 2

static rt_timer_t vs_timer;
static struct rt_event vs_event;
static struct rt_mutex vs_mutex;//��������ݷ��͵Ļ�����
VS_CTL vs_ctl = {0};

u8 vs_dma_status(void)
{
    return spi_dma_status;
}
void vs_spi_dma_init(void)
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
	
	DMA_ClearITPendingBit(DMA1_Stream4,DMA_IT_TCIF4); //��������ж�����λ
	DMA_ClearITPendingBit(DMA1_Stream4,DMA_IT_HTIF4);//����һ������ж�����λ
	DMA_ITConfig(DMA1_Stream4,DMA_IT_TC,ENABLE);//��ȫ�����������ж�Ҳ����4K����
	DMA_ITConfig(DMA1_Stream4,DMA_IT_HT,ENABLE);//�򿪴�����һ�������жϣ�Ҳ���Ǵ�����2K
	
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream4_IRQn;//����DMA�ж�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);
	//DMA_Cmd(DMA1_Stream4, ENABLE);//�򿪸�SPI�Ŀ�������
	//SPI_I2S_DMACmd(SPI2,SPI_I2S_DMAReq_Tx,DISABLE);
}

void vs_spi_dma_stop()
{
    SPI_I2S_DMACmd(SPI2,SPI_I2S_DMAReq_Tx,DISABLE);
    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);//��һ�ֽڷ������������Ƭѡ��
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

void vs_dreq_int(void)//vs�½����ж�
{
    //vs1003_printf("int\n");
    //vs_spi_dma_stop();
}
void vs_spi_init(void)
{ 
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
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    /* CPHA */
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    /* MSB or LSB */
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;//ȫ˫��
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_NSS  = SPI_NSS_Soft;//cs������
    
    /* init SPI */
    SPI_I2S_DeInit(SPI2);
    SPI_Init(SPI2, &SPI_InitStructure);
    SPI_CalculateCRC(SPI2, DISABLE);
     /* Enable SPI_MASTER */
    SPI_Cmd(SPI2, ENABLE);
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

    //SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC,GPIO_PinSource6);
    /* ����EXTI_Line11 */
    EXTI_InitStructure.EXTI_Line = EXTI_Line6;//LINE11
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//�ж��¼�
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //�½��ش��� 
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;//ʹ��LINE0
    //EXTI_Init(&EXTI_InitStructure);//����

	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;//�ⲿ�ж�0
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//��ռ���ȼ�0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;//�����ȼ�2
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//ʹ���ⲿ�ж�ͨ��
    //NVIC_Init(&NVIC_InitStructure);//����
}
s32 vs_dreq_status(void)//vs1003�Ƿ��пռ�������ݣ��ߵ�ƽ:�пռ䣬�͵�ƽ:�޿ռ�
{
    return GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_6);
}
s32 vs_set_status(u8 status)
{
    rt_base_t level;
    level = rt_hw_interrupt_disable();
    vs_ctl.vs_status = status;
    rt_hw_interrupt_enable(level);
}
s32 vs_get_status(void)
{
    return vs_ctl.vs_status;
}
void vs_soft_reset(void)
{
    
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
void vs_spi_write_bytes(unsigned char *data,int len)
{ 
    int i;
    vs_xdcs_cs(0);
    for(i=0;i<len;i++)
    {
        SPI_Write_Byte_vs(data[i]);
    }
    vs_xdcs_cs(1);
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
     vs_xcs_cs(0);//��Ƭѡ��SCI��Ч���������ܶԹ��ܼĴ������ж�д
     SPI_Write_Byte_vs(VS_WRITE_COMMAND);  //д�������0x02   00000010 �����ܼĴ���д������
     SPI_Write_Byte_vs(addr);  //д��Ĵ�����ַ
     SPI_Write_Byte_vs(hdat);  //д����ֽ�
     SPI_Write_Byte_vs(ldat);  //д����ֽ�
     vs_xcs_cs(1); //�ر�Ƭѡ��SCI��Ч
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
    unsigned int temp=0,temp1=0;
    while(!vs_dreq_status());//VS1003��DREQΪ�ߵ�ƽʱ�Ž�������
    vs_xcs_cs(0);//��Ƭѡ��SCI��Ч
    SPI_Write_Byte_vs(VS_READ_COMMAND);  //����������0x03   00000011�����ܼĴ�����������
    temp =SPI_Write_Byte_vs(addr);  //
    temp =SPI_Write_Byte_vs(addr);  //д��Ĵ�����ַ ��������
    temp1 =SPI_Write_Byte_vs(addr);  //д��Ĵ�����ַ ��������
    temp<<=8;
    temp|=temp1;
    vs_xcs_cs(1); //�ر�Ƭѡ��SCI��Ч
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
    vs_reset_cmd(1);   //RES=1 
    rt_thread_delay(1);
    vs_reset_cmd(0);        //RES=0
    rt_thread_delay(1);
    vs_reset_cmd(1);    //Ӳ����λ��XRESET�͵�ƽ��Ч
    rt_thread_delay(1);

    VS_Write_Reg(SPI_MODE  ,0x08,0x04);  //������λ����0�żĴ���д��0x0804   SM_SDINEWΪ1   SM_RESETΪ1
    VS_Write_Reg(SPI_CLOCKF,0x98,0x00);  //ʱ�����ã���3�żĴ���д��0x9800   SC_MULT  Ϊ4   SC_ADD  Ϊ3   SC_FREQΪ0
    VS_Write_Reg(SPI_VOL   ,0, 0);  //�������ã������������������
   
    vs_xdcs_cs(0);	     //������Ƭѡ��ע���ʱXCS������Ƭѡ��Ϊ�ߵ�ƽ��SDI��Ч
    SPI_Write_Byte_vs(0);    //д�����ݣ�����д��4��0�����޹����ݣ������������ݴ���
    SPI_Write_Byte_vs(0);
    SPI_Write_Byte_vs(0);
    SPI_Write_Byte_vs(0);
    vs_xdcs_cs(1);	    //�ر�����Ƭѡ��SDI��Ч
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
    vs_xdcs_cs(0);	   //������Ƭѡ��������SDI����
    for(i=0;i<2048;i++)
    {
        VS_Send_Dat(0);
    }
    vs_xdcs_cs(1);        //�ر�����Ƭѡ
}

void vs1003_init(void)
{
    vs_io_init();
    vs_spi_init();
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
    vs_spi_init();
    VS_Reset();
    VS_Write_Reg(0x00,0x08,0x20);//�������ԣ���0�żĴ���д��0x0820   SM_SDINEWΪ1   SM_TESTΪ1
    while(!vs_dreq_status());   //�ȴ�DREQ��Ϊ�ߵ�ƽ
    vs_xdcs_cs(0);	        //������Ƭѡ SDI��Ч
    SPI_Write_Byte_vs(0x53);//д������8���ֽ�,�������Ҳ���
    SPI_Write_Byte_vs(0xef); 
    SPI_Write_Byte_vs(0x6e);
    SPI_Write_Byte_vs(x);   //����x�����������Ҳ��������Ҳ���Ƶ��   FsIdx (b7~b5):�����ʱ�����   S (b4~b0):���Ҳ���Ծ��   Ƶ��F=Fs X S / 128
    SPI_Write_Byte_vs(0);   //����x=126 (0b 011 11110) FsIdx=011=3   Fs=22050Hz   S=11110=30    F=22050Hz X 30 /128 =5168 Hz
    SPI_Write_Byte_vs(0);
    SPI_Write_Byte_vs(0);
    SPI_Write_Byte_vs(0);
    vs_xdcs_cs(1);
    rt_thread_delay(4000);      //������ʱһ��ʱ�䣬Ϊ����������������
    
    while(!vs_dreq_status());   //�ȴ�DREQ��Ϊ�ߵ�ƽ
    vs_xdcs_cs(0);
    SPI_Write_Byte_vs(0x45);//д������8���ֽڣ��˳����Ҳ���
    SPI_Write_Byte_vs(0x78); 
    SPI_Write_Byte_vs(0x69);
    SPI_Write_Byte_vs(0x74);
    SPI_Write_Byte_vs(0);
    SPI_Write_Byte_vs(0);
    SPI_Write_Byte_vs(0);
    SPI_Write_Byte_vs(0);
    vs_xdcs_cs(1);	    //�ر�����Ƭѡ ��SDI��Ч
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
	    ret = read(fd,vs1003_data_buff,320);//��仺����
	    data_len = 0;
	    while(1)
	    {
	        int i;
	        while(!vs_dreq_status());   //�ȴ�DREQ��Ϊ�ߵ�ƽ
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
    static int tc = 0,ht = 0;
    rt_kprintf("/r tc:%d,ht:%d     ",tc,ht);
	if(DMA_GetITStatus(DMA1_Stream4,DMA_IT_TCIF4))//��������жϱ�־
    {
        //rt_kprintf("tc\n");
        tc++;
        rt_event_send(&vs_event,VS_DMA_TC_EVENT);
    }
	else if(DMA_GetITStatus(DMA1_Stream4,DMA_IT_HTIF4))//����һ������жϱ�־
	{
	    //rt_kprintf("ht\n");
	    ht++;
        rt_event_send(&vs_event,VS_DMA_HT_EVENT);
    }
	DMA_ClearITPendingBit(DMA1_Stream4,DMA_IT_TCIF4 | DMA_IT_HTIF4);
}
u8 time_flag = 10;
static void vs_music_service_task(void *param)
{
    rt_err_t ret;
    rt_uint32_t recved;

    vs1003_init();
    vs_ctl.play_way = 1;
    rt_memset(&vs_event,0,sizeof(struct rt_event));
    ret = rt_event_init(&vs_event, "vs_event", RT_IPC_FLAG_FIFO);
    if(ret!=RT_EOK)
    {
        rt_kprintf("rt_event_init fail\n");
        return ;
    }
    rt_event_send(&vs_event,VS_PLAY_OVER_EVENT);
    ret = rt_mutex_init(&vs_mutex, "vs_mutex", RT_IPC_FLAG_FIFO);
    if(ret!=RT_EOK)
    {
        rt_kprintf("rt_mutex_init fail\n");
        return ;
    }
    while (1)
    {
        if(rt_event_recv(&vs_event,VS_START_EVENT | VS_STOP_EVENT  | VS_PLAY_OVER_EVENT,
                            RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                            rt_tick_from_millisecond(time_flag),&recved)!= RT_EOK)
        {
            if(vs_ctl.fd > 0 && vs_ctl.vs_status == 3)
            {
                
                rt_mutex_take(&vs_mutex,RT_WAITING_FOREVER);
                while(1)
                {
                    if(vs_dreq_status())
                    {
                        int ret;
                        int offset;
                        int write_len;
                        
                        offset = VS_FLIE_BUFFER_LEN - vs_ctl.data_len;
                        write_len = vs_ctl.data_len >= 32 ? 32:vs_ctl.data_len;
                        vs_ctl.data_len -= write_len;
                        vs_spi_write_bytes(&(vs_ctl.data_buffer[offset]),write_len);
                        if(!vs_ctl.data_len)
                        {
                            ret = read(vs_ctl.fd,vs_ctl.data_buffer,VS_FLIE_BUFFER_LEN);
                            vs_ctl.data_len = ret;
                            if(ret <= 0)
                            {
                                rt_kprintf("music over\n");
                                rt_event_send(&vs_event,VS_PLAY_OVER_EVENT);//���͸������Ž������¼�
                                vs_set_status(0);
                                break;
                            }
                        }
                    }
                    else
                    {
                        break;//�͵�ƽ����������������
                    }
                }
                rt_mutex_release(&vs_mutex);
                
            }

            if(vs_ctl.vs_status == 2)
            {
                dev_record_doing();
            }
            continue;
        }
        //����¼�
        rt_kprintf("recvied recved %08x\n",recved);
        if(recved & VS_START_EVENT)
        {
            rt_kprintf("recvied start cmd\n");
            //vs_spi_dma_start();
        }
        if(recved & VS_STOP_EVENT)
        {
            rt_kprintf("recvied stop cmd\n");
            //vs_spi_dma_stop();
        }
        if(recved & VS_PLAY_OVER_EVENT)
        {
            rt_kprintf("play over\n");
            if(vs_ctl.play_way == VS_PLAY_CIRCLE)
            {
                rt_kprintf("play again\n");
                vs_auto_play("/music/");
            }
        }
    }
}
int vs_CheckFileExtname(char* path)
{
	char* temp;
	
	temp = strrchr(path,'.');
	
	temp ++;
	
	if(!rt_strcasecmp(temp,"MP3"))
		return AudioFileType_MP3;
	else if(!rt_strcasecmp(temp,"WAV"))
		return AudioFileType_WAV;
	
	return AudioFileType_ERROR;
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
        rt_kprintf("seekdir\n");
        seekdir(dir,SEEK_SET);
        file_info = readdir(dir);
    }
    if(vs_CheckFileExtname(file_info->d_name) == AudioFileType_ERROR)
    {
        rt_kprintf("vs_CheckFileExtname error\n");
        rt_event_send(&vs_event,VS_PLAY_OVER_EVENT);
        return -1;
    }
    rt_memset(temp_file_name,0,60);
    rt_strncpy(temp_file_name,file_dir_name,rt_strlen(file_dir_name));
    rt_strncpy(temp_file_name+rt_strlen(temp_file_name),file_info->d_name,rt_strlen(file_info->d_name));
    rt_kprintf("��ǰ�ļ�Ϊ=%s\n",temp_file_name);
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
                           RT_VS1003_THREAD_PRIORITY,
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
INIT_APP_EXPORT(vs_music_startup);

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
    int fd;
	char e;
    if (argc < 2)
    {
        music_log("Usage: err\n");
        return;
    }
	
    if(!strncmp(argv[1],"run",rt_strlen("run")))
    {
        int num;
        num = atoi(argv[2]);
        rt_kprintf("num:%d\n",num);
        if(num>0&&num<255)
            time_flag = num;
        return ;
    } 
    if(!strncmp(argv[1],"init",rt_strlen("init")))
    {
        rt_kprintf("recorder init\n");
        dev_record_init(argv[2]);
        return ;
    } 
    if(!strncmp(argv[1],"close",rt_strlen("close")))
    {
        rt_kprintf("recorder close\n");
        dev_record_close();
        return ;
    }
    if(!strncmp(argv[1],"read",rt_strlen("read")))
    {
        int num;
        rt_mutex_take(&vs_mutex,RT_WAITING_FOREVER);
        num = atoi(argv[2]);
        rt_kprintf("VS_Read_Reg:%04x\n",VS_Read_Reg(num));
        
        rt_mutex_release(&vs_mutex);
        return ;
    } 
    if(!strncmp(argv[1],"wr",rt_strlen("wr")))
    {
        int num;
        rt_mutex_take(&vs_mutex,RT_WAITING_FOREVER);
        num = atoi(argv[2]);
        VS_Write_Reg(num,atoi(argv[3]),atoi(argv[4]));
        rt_mutex_release(&vs_mutex);
        return ;
    } 
    if(!strncmp(argv[1],"e",rt_strlen("e")))
    {
        rt_memcpy(&e,argv[2],1);
        rt_kprintf("e event\n");
        switch(e){
        case 'n':
            rt_kprintf("n event\n");
            rt_event_send(&vs_event,VS_PLAY_OVER_EVENT);
            break;
        case 's':
            rt_kprintf("s event\n");
            rt_event_send(&vs_event,VS_START_EVENT);
            break;
        case 't':
            rt_kprintf("t event\n");
            rt_event_send(&vs_event,VS_STOP_EVENT);
            break;
        default:
            break;
				}
        return ;
    }
    if(!strncmp(argv[1],"ctl",rt_strlen("ctl")))
    {
        int temp = atoi(argv[3]);
        if(!strncmp(argv[2],"vol",rt_strlen("vol")))
        {
            vs_audio_ioctl(SPI_VOL,temp,0);
        }
        else if(!strncmp(argv[2],"rec",rt_strlen("rec")))
        {
            vs_audio_ioctl(VS_RECORDER_MODE,temp,0);
        }
        else if(!strncmp(argv[2],"res",rt_strlen("res")))
        {
            vs_audio_ioctl(VS_SOFT_RESET,temp,0);
        }
        else if(!strncmp(argv[2],"diff",rt_strlen("diff")))
        {
            vs_audio_ioctl(VS_SM_DIFF_MODE,temp,0);
        }
        else if(!strncmp(argv[2],"down",rt_strlen("down")))
        {
            vs_audio_ioctl(VS_SM_PDOWN_MODE,temp,0);
        }
        else
        {
            rt_kprintf("err cmd\n");
        }
        return ;
    }
    dev_audio_play(argv[1]);
}
int vs_audio_ioctl(int nCmd ,int lParam ,int wParam)
{
    unsigned char hbat;
    unsigned char lbat;
    int reg_value = 0;
    rt_mutex_take(&vs_mutex,RT_WAITING_FOREVER);
    if(nCmd>=SPI_MODE && nCmd<=SPI_AICTRL3)
    {
        reg_value = VS_Read_Reg(nCmd);
    }
    switch(nCmd)
    {
        case SPI_MODE :
            
        break;
        case SPI_STATUS:
        
        break;
        case SPI_BASS:

        break;
        case SPI_CLOCKF :

        break;
        case SPI_DECODE_TIME :

        break;
        case SPI_AUDATA :

        break;
        case SPI_HDAT0 :

        break;
        case SPI_HDAT1 :

        break;
        case SPI_VOL:
            if(lParam>=0 && lParam<=254)
            {
                hbat = lbat = lParam;
            }
            else
            {
                hbat = lbat = 140;
            }
            VS_Write_Reg(SPI_VOL,hbat,lbat);
        break;
        case VS_SOFT_RESET:
            reg_value = VS_Read_Reg(SPI_MODE);
            hbat = (reg_value >> 8);
            lbat = reg_value | SM_RESET;
            VS_Write_Reg(SPI_MODE,hbat,lbat);
        break;
        case VS_RECORDER_MODE:
            reg_value = VS_Read_Reg(SPI_MODE);
            lbat = reg_value;
            if(lParam)
            {
                hbat = (reg_value >> 8) | 0x30;
            }
            else
            {
                hbat = (reg_value >> 8) & (~0x30);
            }
            VS_Write_Reg(SPI_MODE,hbat,lbat);
        break;
        case VS_SM_DIFF_MODE:
            reg_value = VS_Read_Reg(SPI_MODE);
            hbat = (reg_value >> 8);
            if(lParam)
            {
                lbat = reg_value | 0x01;
            }
            else
            {
                lbat = reg_value & (~0x01);
            }
            VS_Write_Reg(SPI_MODE,hbat,lbat);
        break;
        case VS_SM_PDOWN_MODE:
           reg_value = VS_Read_Reg(SPI_MODE);
           hbat = (reg_value >> 8);
           if(lParam)
           {
               lbat = reg_value | SM_PDOWN;
           }
           else
           {
               lbat = reg_value & (~SM_PDOWN);
           }
           VS_Write_Reg(SPI_MODE,hbat,lbat);
       break;
        default:
            rt_kprintf("cmd  err\n");
            break;
    }
    if(nCmd>=SPI_MODE && nCmd<=SPI_AICTRL3)
    {
        rt_kprintf("read cmd %02x :%04x",nCmd ,VS_Read_Reg(nCmd));
    }
    else
    {
        rt_kprintf("read cmd %02x :%04x",nCmd ,VS_Read_Reg(SPI_MODE));
    }
    
    rt_mutex_release(&vs_mutex);
    return 0;
}
int dev_audio_play(char* file_name)
{
    int fd;
    int ret;
    rt_thread_t tid;
    char temp_file[128];
    
    rt_memset(temp_file,0,128);
    if(file_name[0] != '/')
	{
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
    rt_kprintf("dev_audio_play:%s\n",temp_file);
    vs_set_status(0);//����vsֹͣ
    if(vs_ctl.fd > 0)
    {
        close(vs_ctl.fd);
    }
    if(vs_ctl.play_way)
    {
        rt_memset(&vs_ctl,0,sizeof(VS_CTL));
        vs_ctl.play_way = 1;
    }
    else
    {
        rt_memset(&vs_ctl,0,sizeof(VS_CTL));
    }
    vs_ctl.fd = fd;
    rt_memcpy(vs_ctl.file_name,temp_file,rt_strlen(temp_file));
    ret = read(vs_ctl.fd,vs_ctl.data_buffer,VS_FLIE_BUFFER_LEN);
    rt_kprintf("dev_audio_play read:%d\n",ret);
    vs_ctl.data_len = ret;
    vs_set_status(1);//����vs����
    tid = rt_thread_find("vs1003");
    if(!tid)
    {
        rt_kprintf("dev_audio_play run on vs1003\n");
        vs_music_startup();//����vs1003��������
    }
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
MSH_CMD_EXPORT_ALIAS(vs_music_startup,vs, vs_music_startup vs1003����);
MSH_CMD_EXPORT_ALIAS(vs_music_action,  vscmd,  vscmd);
MSH_CMD_EXPORT_ALIAS(vs_1003_cmd, cmd, vsspi);
FINSH_FUNCTION_EXPORT(vs_sin_test, vs_sin_test);
MSH_CMD_EXPORT(vs_music_test, vs_music_test);
#endif
