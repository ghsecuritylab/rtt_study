/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2017-06-08     tanek        first implementation
 */

#include "drv_lcd.h"
#include <finsh.h>

//#define DEBUG

#ifdef DEBUG
#define DEBUG_PRINTF(...)   rt_kprintf(__VA_ARGS__)
#else
#define DEBUG_PRINTF(...)   
#endif

typedef struct
{
	rt_uint16_t width;			//LCD ���
	rt_uint16_t height;			//LCD �߶�
	rt_uint16_t id;				//LCD ID
	rt_uint8_t  dir;			//���������������ƣ�0��������1��������	
	rt_uint16_t	wramcmd;		//��ʼдgramָ��
	rt_uint16_t setxcmd;		//����x����ָ��
	rt_uint16_t setycmd;		//����y����ָ�� 
} lcd_info_t;

typedef struct
{
	volatile rt_uint16_t reg;
	volatile rt_uint16_t ram;
} lcd_ili9341_t;
//A12��Ϊ��������������  ����ʱSTM32�ڲ�������һλ����	    
#define LCD_ILI9341_BASE        ((rt_uint32_t)(0x6C000000 | 0x00001FFE))
#define ili9341					((lcd_ili9341_t *) LCD_ILI9341_BASE)

static lcd_info_t lcddev;

void delay_us(rt_uint32_t nus)
{
	//rt_thread_delay(1);
	while (nus--) {
		__NOP();
	}
}

void delay_ms(rt_uint32_t nms)
{
	//rt_thread_delay((RT_TICK_PER_SECOND * nms + 999) / 1000);
	while (nms--)
	{
		int i;
		for (i = 0; i < 10000; i++)
		{
			__NOP();
		}
	}
}

static void ili9341_write_reg(rt_uint16_t regval)
{
	ili9341->reg = regval;
}
static void WriteComm(rt_uint16_t regval)
{
	ili9341->reg = regval;
}
static void WriteData(rt_uint16_t regval)
{
	ili9341->ram = regval;
}

static void ili9341_write_data(rt_uint16_t data)
{
	ili9341->ram = data;
}

rt_uint16_t ili9341_read_ram(void)
{
	return ili9341->ram;
}

static void ili9341_write_reg_with_value(rt_uint16_t reg, rt_uint16_t regValue)
{
	ili9341->reg = reg;
	ili9341->ram = regValue;
}

static void ili9341_write_ram_prepare(void)
{
	ili9341->reg = lcddev.wramcmd;
}

rt_uint16_t ili9341_bgr2rgb(rt_uint16_t value)
{
	rt_uint16_t  red, green, blue;

	blue = (value >> 0) & 0x1f;
	green = (value >> 5) & 0x3f;
	red = (value >> 11) & 0x1f;

	return (blue << 11) + (green << 5) + (red << 0);
}

static void ili9341_set_cursor(rt_uint16_t Xpos, rt_uint16_t Ypos)
{
	ili9341_write_reg(lcddev.setxcmd);
	ili9341_write_data(Xpos >> 8); 
	ili9341_write_data(Xpos & 0XFF);

	ili9341_write_reg(lcddev.setycmd);
	ili9341_write_data(Ypos >> 8); 
	ili9341_write_data(Ypos & 0XFF);
}
  	   
static void ili9341_set_scan_direction(rt_uint8_t dir)
{
	rt_uint16_t regval = 0;
	rt_uint16_t dirreg = 0;
	rt_uint16_t temp;

	switch (dir)
	{
	case L2R_U2D://������,���ϵ���
		regval |= (0 << 7) | (0 << 6) | (0 << 5);
		break;
	case L2R_D2U://������,���µ���
		regval |= (1 << 7) | (0 << 6) | (0 << 5);
		break;
	case R2L_U2D://���ҵ���,���ϵ���
		regval |= (0 << 7) | (1 << 6) | (0 << 5);
		break;
	case R2L_D2U://���ҵ���,���µ���
		regval |= (1 << 7) | (1 << 6) | (0 << 5);
		break;
	case U2D_L2R://���ϵ���,������
		regval |= (0 << 7) | (0 << 6) | (1 << 5);
		break;
	case U2D_R2L://���ϵ���,���ҵ���
		regval |= (0 << 7) | (1 << 6) | (1 << 5);
		break;
	case D2U_L2R://���µ���,������
		regval |= (1 << 7) | (0 << 6) | (1 << 5);
		break;
	case D2U_R2L://���µ���,���ҵ���
		regval |= (1 << 7) | (1 << 6) | (1 << 5);
		break;
	}

	dirreg = 0X36;
	ili9341_write_reg_with_value(dirreg, regval);

	if (regval & 0X20)
	{
		if (lcddev.width < lcddev.height)//����X,Y
		{
			temp = lcddev.width;
			lcddev.width = lcddev.height;
			lcddev.height = temp;
		}
	}
	else
	{
		if (lcddev.width > lcddev.height)//����X,Y
		{
			temp = lcddev.width;
			lcddev.width = lcddev.height;
			lcddev.height = temp;
		}
	}
	
	ili9341_write_reg(lcddev.setxcmd);
	ili9341_write_data(0);
	ili9341_write_data(0);
	ili9341_write_data((lcddev.width - 1) >> 8);
	ili9341_write_data((lcddev.width - 1) & 0XFF);

	ili9341_write_reg(lcddev.setycmd);
	ili9341_write_data(0);
	ili9341_write_data(0);
	ili9341_write_data((lcddev.height - 1) >> 8);
	ili9341_write_data((lcddev.height - 1) & 0XFF);
}

void ili9341_set_backlight(rt_uint8_t pwm)
{
	ili9341_write_reg(0xBE);
	ili9341_write_data(0x05);
	ili9341_write_data(pwm*2.55);
	ili9341_write_data(0x01);
	ili9341_write_data(0xFF);
	ili9341_write_data(0x00);
	ili9341_write_data(0x00);
}
//�������ṩ�����������������
void BlockWrite(unsigned int Xstart,unsigned int Xend,unsigned int Ystart,unsigned int Yend) 
{
	WriteComm(0x2a);   
	WriteData(Xstart>>8);
	WriteData(Xstart&0xff);
	WriteData(Xend>>8);
	WriteData(Xend&0xff);

	WriteComm(0x2b);   
	WriteData(Ystart>>8);
	WriteData(Ystart&0xff);
	WriteData(Yend>>8);
	WriteData(Yend&0xff);
	
	WriteComm(0x2c);
}

void Lcd_ColorBox(u16 xStart,u16 yStart,u16 xLong,u16 yLong,u16 Color)
{
	u32 temp;

	BlockWrite(xStart,xStart+xLong-1,yStart,yStart+yLong-1);
	for (temp=0; temp<xLong*yLong; temp++)
	{
		ili9341->ram=Color; 
	}
}

void LCD_Clear(u16 color)
{
	Lcd_ColorBox(0,0,LCD_WIDTH,LCD_HIGH,color);
}

void ili9341_set_display_direction(rt_uint8_t dir)
{

	lcddev.wramcmd = 0X2C;
	lcddev.setxcmd = 0X2A;
	lcddev.setycmd = 0X2B;

	//ili9341_set_scan_direction(L2R_D2U);
}
void LCD_FSMC_Config(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;
    FSMC_NORSRAMTimingInitTypeDef  readWriteTiming; 
    FSMC_NORSRAMTimingInitTypeDef  writeTiming;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_GPIOE|RCC_AHB1Periph_GPIOF|RCC_AHB1Periph_GPIOG, ENABLE);
    RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC,ENABLE);//ʹ��FSMCʱ��  
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_14;        //PF10 �������,���Ʊ���
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;     //���ģʽ
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;    //�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;      //����
    GPIO_Init(GPIOF, &GPIO_InitStructure);            //��ʼ��PF10 
      
    GPIO_InitStructure.GPIO_Pin = (3<<0)|(3<<4)|(7<<8)|(3<<14); 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;      //�������
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;    //�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;      //����
    GPIO_Init(GPIOD, &GPIO_InitStructure);            //��ʼ��  
    
    GPIO_InitStructure.GPIO_Pin = (0X1FF<<7);         //PE7~15,AF OUT
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;      //�������
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;    //�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;      //����
    GPIO_Init(GPIOE, &GPIO_InitStructure);            //��ʼ��  

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;         //PG2
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;      //�������
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;    //�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;      //����
    GPIO_Init(GPIOG, &GPIO_InitStructure);            //��ʼ��  

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;        //PG12
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;      //�������
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;    //�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;      //����
    GPIO_Init(GPIOG, &GPIO_InitStructure);            //��ʼ�� 

    GPIO_PinAFConfig(GPIOD,GPIO_PinSource0,GPIO_AF_FSMC); 
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource1,GPIO_AF_FSMC); 
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource4,GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource5,GPIO_AF_FSMC); 
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource8,GPIO_AF_FSMC); 
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource9,GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource10,GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource14,GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource15,GPIO_AF_FSMC); 
   
    GPIO_PinAFConfig(GPIOE,GPIO_PinSource7,GPIO_AF_FSMC);  
    GPIO_PinAFConfig(GPIOE,GPIO_PinSource8,GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE,GPIO_PinSource9,GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE,GPIO_PinSource10,GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE,GPIO_PinSource11,GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE,GPIO_PinSource12,GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE,GPIO_PinSource13,GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE,GPIO_PinSource14,GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE,GPIO_PinSource15,GPIO_AF_FSMC); 
   
    GPIO_PinAFConfig(GPIOG,GPIO_PinSource2,GPIO_AF_FSMC); 
    GPIO_PinAFConfig(GPIOG,GPIO_PinSource12,GPIO_AF_FSMC);   

#if 0
    readWriteTiming.FSMC_AddressSetupTime = 0XF;   //��ַ����ʱ�䣨ADDSET�� 16��HCLK 1/168M=6ns*16=96ns  
    readWriteTiming.FSMC_AddressHoldTime = 0x00;   //��ַ����ʱ�䣨ADDHLD��
    readWriteTiming.FSMC_DataSetupTime = 60;           //���ݱ���ʱ�� 60��HCLK = 6*60=360ns
    readWriteTiming.FSMC_BusTurnAroundDuration = 0x00;
    readWriteTiming.FSMC_CLKDivision = 0x00;
    readWriteTiming.FSMC_DataLatency = 0x00;
    readWriteTiming.FSMC_AccessMode = FSMC_AccessMode_A;   
    
    writeTiming.FSMC_AddressSetupTime =8;        //��ַ����ʱ�䣨ADDSET��9��HCLK =54ns 
    writeTiming.FSMC_AddressHoldTime = 0x00;   //��ַ����ʱ�� 
    writeTiming.FSMC_DataSetupTime = 7;            //���ݱ���ʱ�� 6ns*9��HCLK=54ns
    writeTiming.FSMC_BusTurnAroundDuration = 0x00;
    writeTiming.FSMC_CLKDivision = 0x00;
    writeTiming.FSMC_DataLatency = 0x00;
    writeTiming.FSMC_AccessMode = FSMC_AccessMode_A;   
#else
    readWriteTiming.FSMC_AddressSetupTime = 0x00;	 //��ַ����ʱ�䣨ADDSET��Ϊ1��HCLK 1/36M=27ns
    readWriteTiming.FSMC_AddressHoldTime = 0x00;	 //��ַ����ʱ�䣨ADDHLD��ģʽAδ�õ�	
    readWriteTiming.FSMC_DataSetupTime = 0x0b;		 ////���ݱ���ʱ�䣨DATAST��Ϊ9��HCLK 6*9=54ns	 	 
    readWriteTiming.FSMC_BusTurnAroundDuration = 0x00;
    readWriteTiming.FSMC_CLKDivision = 0x00;
    readWriteTiming.FSMC_DataLatency = 0x00;
    readWriteTiming.FSMC_AccessMode = FSMC_AccessMode_A;	 //ģʽA 
#endif
    FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM4;  
    FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable; 
    FSMC_NORSRAMInitStructure.FSMC_MemoryType =FSMC_MemoryType_SRAM;
    FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;   //���ݿ��Ϊ16bit   
    FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode =FSMC_BurstAccessMode_Disable;
    FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
    FSMC_NORSRAMInitStructure.FSMC_AsynchronousWait=FSMC_AsynchronousWait_Disable; 
    FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;   
    FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;  
    FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;    //дʹ��
    FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;   
    FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Enable;      //��дʹ�ò�ͬ��ʱ��
    FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable; 
    FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &readWriteTiming;     //��дʱ��
    FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &readWriteTiming;             //дʱ��
    FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure);  //��ʼ��FSMC
    FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM4, ENABLE);  //ʹ��Bank1           
    delay_ms(50); 
}
void lcd_reset(void)
{
    GPIO_ResetBits(GPIOF, GPIO_Pin_14);
     delay_ms(20);					   
     GPIO_SetBits(GPIOF, GPIO_Pin_14 );		 	 
     delay_ms(20);	
}

void _lcd_low_level_init(void)
{
    //LCD_FSMC_Config();
    lcd_reset();
    lcddev.height = LCD_HIGH;
    lcddev.width  = LCD_WIDTH;
    //��������дʱ����ƼĴ�����ʱ��   	 							    
	FSMC_Bank1E->BWTR[6]&=~(0XF<<0); //��ַ����ʱ������ 	 
	FSMC_Bank1E->BWTR[6]&=~(0XF<<8); //���ݱ���ʱ������
	FSMC_Bank1E->BWTR[6]|=3<<0;		   //��ַ����ʱ��Ϊ3��HCLK =18ns  	 
	FSMC_Bank1E->BWTR[6]|=2<<8;    	 //���ݱ���ʱ��Ϊ6ns*3��HCLK=18ns
	delay_ms(50);
	//************* Start Initial Sequence **********//
    WriteComm(0xFF); // EXTC Command Set enable register 
    WriteData(0xFF); 
    WriteData(0x98); 
    WriteData(0x06); 

    WriteComm(0xBA); // SPI Interface Setting 
    WriteData(0xE0); 

    WriteComm(0xBC); // GIP 1 
    WriteData(0x03); 
    WriteData(0x0F); 
    WriteData(0x63); 
    WriteData(0x69); 
    WriteData(0x01); 
    WriteData(0x01); 
    WriteData(0x1B); 
    WriteData(0x11); 
    WriteData(0x70); 
    WriteData(0x73); 
    WriteData(0xFF); 
    WriteData(0xFF); 
    WriteData(0x08); 
    WriteData(0x09); 
    WriteData(0x05); 
    WriteData(0x00);
    WriteData(0xEE); 
    WriteData(0xE2); 
    WriteData(0x01); 
    WriteData(0x00);
    WriteData(0xC1); 

    WriteComm(0xBD); // GIP 2 
    WriteData(0x01); 
    WriteData(0x23); 
    WriteData(0x45); 
    WriteData(0x67); 
    WriteData(0x01); 
    WriteData(0x23); 
    WriteData(0x45); 
    WriteData(0x67); 

    WriteComm(0xBE); // GIP 3 
    WriteData(0x00); 
    WriteData(0x22); 
    WriteData(0x27); 
    WriteData(0x6A); 
    WriteData(0xBC); 
    WriteData(0xD8); 
    WriteData(0x92); 
    WriteData(0x22); 
    WriteData(0x22); 

    WriteComm(0xC7); // Vcom 
    WriteData(0x1E);
     
    WriteComm(0xED); // EN_volt_reg 
    WriteData(0x7F); 
    WriteData(0x0F); 
    WriteData(0x00); 

    WriteComm(0xC0); // Power Control 1
    WriteData(0xE3); 
    WriteData(0x0B); 
    WriteData(0x00);
     
    WriteComm(0xFC);
    WriteData(0x08); 

    WriteComm(0xDF); // Engineering Setting 
    WriteData(0x00); 
    WriteData(0x00); 
    WriteData(0x00); 
    WriteData(0x00); 
    WriteData(0x00); 
    WriteData(0x02); 

    WriteComm(0xF3); // DVDD Voltage Setting 
    WriteData(0x74); 

    WriteComm(0xB4); // Display Inversion Control 
    WriteData(0x00); 
    WriteData(0x00); 
    WriteData(0x00); 

    WriteComm(0xF7); // 480x854
    WriteData(0x81); 

    WriteComm(0xB1); // Frame Rate 
    WriteData(0x00); 
    WriteData(0x10); 
    WriteData(0x14); 

    WriteComm(0xF1); // Panel Timing Control 
    WriteData(0x29); 
    WriteData(0x8A); 
    WriteData(0x07); 

    WriteComm(0xF2); //Panel Timing Control 
    WriteData(0x40); 
    WriteData(0xD2); 
    WriteData(0x50); 
    WriteData(0x28); 

    WriteComm(0xC1); // Power Control 2 
    WriteData(0x17);
    WriteData(0X85); 
    WriteData(0x85); 
    WriteData(0x20); 

    WriteComm(0xE0); 
    WriteData(0x00); //P1 
    WriteData(0x0C); //P2 
    WriteData(0x15); //P3 
    WriteData(0x0D); //P4 
    WriteData(0x0F); //P5 
    WriteData(0x0C); //P6 
    WriteData(0x07); //P7 
    WriteData(0x05); //P8 
    WriteData(0x07); //P9 
    WriteData(0x0B); //P10 
    WriteData(0x10); //P11 
    WriteData(0x10); //P12 
    WriteData(0x0D); //P13 
    WriteData(0x17); //P14 
    WriteData(0x0F); //P15 
    WriteData(0x00); //P16 

    WriteComm(0xE1); 
    WriteData(0x00); //P1 
    WriteData(0x0D); //P2 
    WriteData(0x15); //P3 
    WriteData(0x0E); //P4 
    WriteData(0x10); //P5 
    WriteData(0x0D); //P6 
    WriteData(0x08); //P7 
    WriteData(0x06); //P8 
    WriteData(0x07); //P9 
    WriteData(0x0C); //P10 
    WriteData(0x11); //P11 
    WriteData(0x11); //P12 
    WriteData(0x0E); //P13 
    WriteData(0x17); //P14 
    WriteData(0x0F); //P15 
    WriteData(0x00); //P16

    #if 0 /*xqy 2018-6-18*/
    uart_printf("����ֵ=%08x\n",LCD_ReadReg(0x52));
    LCD_WriteReg(0x51,125);
    #endif
    WriteComm(0x35); //Tearing Effect ON 
    WriteData(0x00); 

    WriteComm(0x3A); //���������������Ƕ���λ��ʾ��d6~d4 101:16 110:18 111:24λ
	WriteData(0x55);
	
	WriteComm(0x36);//����ɨ�跽��ģ����ң�����
	if(1)
	{
	    WriteData(0x00);//������0x60,������0x00 
	}
	else
	{
	    WriteData(0x60);//������0x60,������0x00 
	}
    LCD_BACK=1;					//��������
    WriteComm(0x11); //Exit Sleep 
    delay_ms(120); 
    WriteComm(0x29); // Display On 
    delay_ms(30);
	//ili9341_set_backlight(255);//��������
	ili9341_set_display_direction(1);
	LCD_Clear(RED);
	
}

void draw_screen(u16 color)
{
    LCD_Clear(color);
}
static u32 lcd_dma_tick_start = 0;
static u32 lcd_dma_tick_end = 0;
#define HEAP_SIZE_SRAM 500//*1024
u8 sram_heap[HEAP_SIZE_SRAM]={0};// __attribute__((aligned(4)));
static u16 color_data __attribute__((aligned(4))) = 0;

u8 src_addr[500*1024] = {0};
u8 dst_addr[100] = {0};

void fmsc_dma_write_data(u16* lcd_data,u32 len,u8 flag/*lcd_data ��ַ�Ƿ����� 0����1����*/)
{
    DMA_InitTypeDef            DMA_InitStructure;
	NVIC_InitTypeDef				NVIC_InitStructure;
	
    rt_kprintf("fmsc_dma_init enter\n");
	DMA_DeInit(DMA2_Stream1);//���֮ǰ��stream4�ϵ������жϱ�־
	//����DMA��ÿ��8����������ÿ����������8��ͨ��
    while (DMA_GetCmdStatus(DMA2_Stream1) != DISABLE){}//�ȴ�DMA������ 
    
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;  //ͨ��ѡ��
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)lcd_data;//�ⲿram���ݵ�ַ
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&(ili9341->ram);//LCD�������ݵ�ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToMemory;//�ڴ浽�ڴ�
	DMA_InitStructure.DMA_BufferSize = len;//
	if(flag)
	    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Enable;//FMSC ram����
    else
        DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//FMSC ram����
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;//LCD->RAM�Զ����ӹر�
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//���ִ���16λ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;//Ӧ��������ģʽ
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;//���ȼ��ǳ���
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//�洢��ͻ�����δ���
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//����ͻ�����δ���
	DMA_Init(DMA2_Stream1, &DMA_InitStructure);
	
	DMA_ClearITPendingBit(DMA2_Stream1,DMA_IT_TCIF1); 
	DMA_ITConfig(DMA2_Stream1,DMA_IT_TC,ENABLE);
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream1_IRQn;//����DMA�ж�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);
	DMA_Cmd(DMA2_Stream1, ENABLE);
	lcd_dma_tick_start = rt_tick_get();
	rt_kprintf("fmsc_dma start\n");
}

void fmsc_dma_t(u8 flag)
{
    DMA_InitTypeDef            DMA_InitStructure;
	NVIC_InitTypeDef				NVIC_InitStructure;
    u8 buff_1 = 0xf1;
	rt_memset(src_addr,0,HEAP_SIZE_SRAM);
    rt_kprintf("fmsc_dma_init enter\n");
    BlockWrite(0,0+480-1,400,0+845-1);
	DMA_DeInit(DMA2_Stream1);//���֮ǰ��stream4�ϵ������жϱ�־
	//����DMA��ÿ��8����������ÿ����������8��ͨ��
    while (DMA_GetCmdStatus(DMA2_Stream1) != DISABLE){}//�ȴ�DMA������ 
    
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;  //ͨ��ѡ��
	if(flag)
	    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)src_addr;//�ⲿram���ݵ�ַ
    else
        DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&buff_1;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&(ili9341->ram);//(uint32_t)sram_heap;//LCD�������ݵ�ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToMemory;//�ڴ浽�ڴ�
	DMA_InitStructure.DMA_BufferSize = 65535;//
	if(flag)
	    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Enable;//FMSC ram����
    else
        DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//FMSC ram����
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;//LCD->RAM�Զ����ӹر�
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//���ִ���16λ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;//Ӧ��������ģʽ
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;//���ȼ��ǳ���
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//�洢��ͻ�����δ���
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//����ͻ�����δ���
	DMA_Init(DMA2_Stream1, &DMA_InitStructure);
	
	DMA_ClearITPendingBit(DMA2_Stream1,DMA_IT_TCIF1); 
	DMA_ITConfig(DMA2_Stream1,DMA_IT_TC,ENABLE);
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream1_IRQn;//����DMA�ж�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);
	DMA_Cmd(DMA2_Stream1, ENABLE);
	lcd_dma_tick_start = rt_tick_get();
	rt_kprintf("fmsc_dma start\n");
}

void DMA2_Stream1_IRQHandler(void)
{
    rt_interrupt_enter();
    //rt_sem_release(dma_int);
    lcd_dma_tick_end = rt_tick_get();
    rt_kprintf("DMA2_Stream1_IRQHandler tick:%d\n",lcd_dma_tick_end - lcd_dma_tick_start);
	if(DMA_GetITStatus(DMA2_Stream1,DMA_IT_TCIF1))//��������жϱ�־
    {
        rt_kprintf("ȫ���\n");
    }
	else if(DMA_GetITStatus(DMA2_Stream1,DMA_IT_HTIF1))//����һ������жϱ�־
	{
        rt_kprintf("�����\n");
    }
	DMA_ClearITPendingBit(DMA2_Stream1,DMA_IT_TCIF1);
	printf_format(sram_heap,100);
	rt_interrupt_leave();
}
void sram_zroe_speed_test(void)
{
    u32 s_tick = 0;
    u32 e_tick = 0;
    u8  data_1 = 0;
	lcd_dma_tick_start = rt_tick_get();
    s_tick = rt_tick_get();
    rt_memset(sram_heap,0,HEAP_SIZE_SRAM);
    e_tick = rt_tick_get();
    rt_kprintf("--------------------sram_zroe_speed_test tick :%d ,end :%d  start:%d\n",e_tick - s_tick,e_tick,s_tick);
}
void fmsc_dma_test(u16 color,u8 flag,u32 len)//flag 0:�ڲ�ram�еĵ�ɫ����������,1:�ⲿram�еĵ�ɫ 2:�ⲿram����
{
    u16 color_1 = color;
    if(flag ==0)
    {
        rt_kprintf("�ڲ�ram������\n");
        BlockWrite(0,0+480-1,0,0+845-1);
        fmsc_dma_write_data(&color_1,480*845,0);
    }
    else if(flag ==1 )
    {
        rt_kprintf("�ⲿram������\n");
        color_data = color;
        BlockWrite(0,0+480-1,0,0+845-1);
        fmsc_dma_write_data(&color_data,len,0);
    }
    else if(flag == 2)
    {
        u32 e_tick = 0;
        u32 s_tick = rt_tick_get();
        rt_memset(src_addr,0x0,2*480*30);
        rt_memset(src_addr+2*480*30,0xff,2*480*30);
        e_tick = rt_tick_get();
        rt_kprintf("memset tick :%d\n",e_tick - s_tick);
        BlockWrite(0,0+480-1,100,0+845-1);
        fmsc_dma_write_data((u16*)src_addr,len,1);
    }
    return ;
}
static rt_err_t lcd_init(rt_device_t dev)
{
	return RT_EOK;
}

static rt_err_t lcd_open(rt_device_t dev, rt_uint16_t oflag)
{
	return RT_EOK;
}

static rt_err_t lcd_close(rt_device_t dev)
{
	return RT_EOK;
}

static rt_err_t lcd_control(rt_device_t dev, int cmd, void *args)
{
	switch (cmd)
	{
	case RTGRAPHIC_CTRL_GET_INFO:
	{
		struct rt_device_graphic_info *info;

		info = (struct rt_device_graphic_info*) args;
		RT_ASSERT(info != RT_NULL);

		info->bits_per_pixel = 16;
		info->pixel_format = RTGRAPHIC_PIXEL_FORMAT_RGB565;
		info->framebuffer = RT_NULL;
		info->width = 240;
		info->height = 320;
	}
	break;

	case RTGRAPHIC_CTRL_RECT_UPDATE:
		/* nothong to be done */
		break;

	default:
		break;
	}

	return RT_EOK;
}

static void ili9341_lcd_set_pixel(const char* pixel, int x, int y)
{
	ili9341_set_cursor(x, y);
	ili9341_write_ram_prepare();
	ili9341->ram = *(uint16_t *)pixel;
}


#ifdef RT_USING_FINSH
static void lcd_set_pixel(uint16_t color, int x, int y)
{
	rt_kprintf("lcd set pixel, color: %X, x: %d, y: %d", color, x, y);
	ili9341_lcd_set_pixel((const char *)&color, x, y);
}
FINSH_FUNCTION_EXPORT(lcd_set_pixel, set pixel in lcd display);
FINSH_FUNCTION_EXPORT(draw_screen, draw_screen);
FINSH_FUNCTION_EXPORT(fmsc_dma_t, fmsc_dma_t);
FINSH_FUNCTION_EXPORT(fmsc_dma_test, void fmsc_dma_test u16 color u8 flag flag 0:�ڲ�ram�еĵ�ɫ���������� 1:�ⲿram�еĵ�ɫ 2:�ⲿram���� );


#endif

static void ili9341_lcd_get_pixel(char* pixel, int x, int y)
{
	rt_uint16_t red = 0;
	rt_uint16_t green = 0;
	rt_uint16_t blue = 0;

	if (x >= lcddev.width || y >= lcddev.height)
	{
		*(rt_uint16_t*)pixel = 0;
		return;
	}

	ili9341_set_cursor(x, y);

	ili9341_write_reg(0X2E);
	ili9341_read_ram();
	red = ili9341_read_ram();
	delay_us(2);

	blue = ili9341_read_ram();
	green = red & 0XFF;

	*(rt_uint16_t*)pixel = (((red >> 11) << 11) | ((green >> 10) << 5) | (blue >> 11));
}
#ifdef RT_USING_FINSH
static void lcd_get_pixel(int x, int y)
{
	uint16_t pixel;
	ili9341_lcd_get_pixel((char *)&pixel, x, y);
	rt_kprintf("lcd get pixel, pixel: 0x%X, x: %d, y: %d", pixel, x, y);
}
FINSH_FUNCTION_EXPORT(lcd_get_pixel, get pixel in lcd display);
#endif

static void ili9341_lcd_draw_hline(const char* pixel, int x1, int x2, int y)
{
	ili9341_set_cursor(x1, y);
	ili9341_write_ram_prepare();

	for (; x1 < x2; x1++)
	{
		ili9341->ram = *(uint16_t *)pixel;
	}
}
#ifdef RT_USING_FINSH
static void lcd_draw_hline(uint16_t pixel, int x1, int x2, int y)
{
	ili9341_lcd_draw_hline((const char *)&pixel, x1, x2, y);
	rt_kprintf("lcd draw hline, pixel: 0x%X, x1: %d, x2: %d, y: %d", pixel, x1, x2, y);
}
FINSH_FUNCTION_EXPORT(lcd_draw_hline, draw hline in lcd display);
#endif

static void ili9341_lcd_draw_vline(const char* pixel, int x, int y1, int y2)
{
	for (; y1 < y2; y1++)
	{
		ili9341_lcd_set_pixel(pixel, x, y1);  //write red data
	}
}
#ifdef RT_USING_FINSH
static void lcd_draw_vline(uint16_t pixel, int x, int y1, int y2)
{
	ili9341_lcd_draw_vline((const char *)&pixel, x, y1, y2);
	rt_kprintf("lcd draw hline, pixel: 0x%X, x: %d, y: %d", pixel, y1, y2);
}
FINSH_FUNCTION_EXPORT(lcd_draw_vline, draw vline in lcd display);
#endif

static void ili9341_lcd_blit_line(const char* pixels, int x, int y, rt_size_t size)
{
	rt_uint16_t *ptr = (rt_uint16_t*)pixels;

	ili9341_set_cursor(x, y);
	ili9341_write_ram_prepare();

	while (size--)
	{
		ili9341->ram = *ptr++;
	}
}
#ifdef RT_USING_FINSH
#define LINE_LEN 30
static void lcd_blit_line(int x, int y)
{
	uint16_t pixels[LINE_LEN];
	int i;

	for (i = 0; i < LINE_LEN; i++)
	{
		pixels[i] = i * 40 + 50;
	}

	ili9341_lcd_blit_line((const char *)pixels, x, y, LINE_LEN);
	rt_kprintf("lcd blit line, x: %d, y: %d", x, y);
}
FINSH_FUNCTION_EXPORT(lcd_blit_line, draw blit line in lcd display);
#endif

static int rt_hw_lcd_init(void)
{
	_lcd_low_level_init();

	static struct rt_device lcd_device;

	static struct rt_device_graphic_ops ili9341_ops =
	{
		ili9341_lcd_set_pixel,
		ili9341_lcd_get_pixel,
		ili9341_lcd_draw_hline,
		ili9341_lcd_draw_vline,
		ili9341_lcd_blit_line
	};

	/* register lcd device */
	lcd_device.type = RT_Device_Class_Graphic;
	lcd_device.init = lcd_init;
	lcd_device.open = lcd_open;
	lcd_device.close = lcd_close;
	lcd_device.control = lcd_control;
	lcd_device.read = RT_NULL;
	lcd_device.write = RT_NULL;

	lcd_device.user_data = &ili9341_ops;

	/* register graphic device driver */
	rt_device_register(&lcd_device, "lcd",
		RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE);

	return 0;
}
INIT_BOARD_EXPORT(rt_hw_lcd_init);
