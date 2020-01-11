#include "spblcd.h"
#include "usart.h"
#include "sram_variable.h"
#include <rtdef.h>   
#include "drv_lcd.h"  
#include <rtthread.h>

//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//SPBLCD ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/7/20
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	


#define RT_LCD_THREAD_PRIORITY 4

extern lcd_info_t lcddev;
static rt_sem_t lcd_dma_int = RT_NULL;

//��ָ��λ�û���.
//x,y:����
//color:��ɫ.
void slcd_draw_point(u16 x,u16 y,u16 color)
{	 
	sramlcdbuf[(y-1)*lcddev.width + x -1]=color;
}
//��ȡָ��λ�õ����ɫֵ
//x,y:����
//����ֵ:��ɫ
u16 slcd_read_point(u16 x,u16 y)
{
	return sramlcdbuf[(y-1)*lcddev.width + x -1];
} 
//�����ɫ
//x,y:��ʼ����
//width��height����Ⱥ͸߶�
//*color����ɫ����
void slcd_fill_color(u16 x,u16 y,u16 width,u16 height,u16 *color)
{   
	u16 i,j; 
 	for(i=0;i<height;i++)
	{
		for(j=0;j<width;j++)
		{
			slcd_draw_point(x+j,y+i,*color++);
		}	
	}	
} 
void DMA2_Stream0_IRQHandler(void)
{
	DMA_ClearITPendingBit(DMA2_Stream0,DMA_IT_TCIF0);
	rt_sem_release(lcd_dma_int);
}

//SRAM --> LCD_RAM dma����
//16λ,�ⲿSRAM���䵽LCD_RAM. 
void slcd_dma_init(void)
{  
    NVIC_InitTypeDef				NVIC_InitStructure;
	RCC->AHB1ENR|=1<<22;		//DMA2ʱ��ʹ��  
	while(DMA2_Stream0->CR&0X01);//�ȴ�DMA2_Stream0������ 
	DMA2->LIFCR|=0X3D<<6*0;		//���ͨ��0�������жϱ�־
	DMA2_Stream0->FCR=0X0000021;//����ΪĬ��ֵ	
	
	DMA2_Stream0->PAR=0;		//�ݲ�����
	DMA2_Stream0->M0AR=(u32)&ili9341->ram;	//Ŀ���ַΪLCD_RAM
	DMA2_Stream0->M1AR=0;		//��������
	DMA2_Stream0->NDTR=0;		//��ʱ���ó���Ϊ0
	DMA2_Stream0->CR=0;			//��ȫ����λCR�Ĵ���ֵ  
	DMA2_Stream0->CR|=2<<6;		//�洢�����洢��ģʽ 
	DMA2_Stream0->CR|=0<<8;		//��ͨģʽ
	DMA2_Stream0->CR|=1<<9;		//��������ģʽ
	DMA2_Stream0->CR|=0<<10;	//�洢��������ģʽ
	DMA2_Stream0->CR|=1<<11;	//�������ݳ���:16λ
	DMA2_Stream0->CR|=1<<13;	//�洢�����ݳ���:16λ
	DMA2_Stream0->CR|=2<<16;	//�����ȼ�
	DMA2_Stream0->CR|=0<<18;	//������ģʽ
	DMA2_Stream0->CR|=0<<21;	//����ͻ�����δ���
	DMA2_Stream0->CR|=0<<23;	//�洢��ͻ�����δ���
	DMA2_Stream0->CR|=0<<25;	//ѡ��ͨ��0 
	
	DMA2_Stream0->FCR&=~(1<<2);	//��ʹ��FIFOģʽ
	DMA2_Stream0->FCR&=~(3<<0);	//��FIFO ����  

    DMA_ClearITPendingBit(DMA2_Stream0,DMA_IT_TCIF0); 
	DMA_ITConfig(DMA2_Stream0,DMA_IT_TC,ENABLE);
	//DMA_ITConfig(DMA1_Stream5,DMA_IT_HT,ENABLE);
	//DMA_ITConfig(DMA1_Stream5,DMA_IT_TC|DMA_IT_HT,ENABLE);//DMA_IT_TC|DMA_IT_HT,ENABLE);//����DMA�жϣ���������ж�����/����һ���ж�����
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;//����DMA�ж�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);
	DMA_Cmd(DMA2_Stream0, ENABLE);
	
} 
INIT_BOARD_EXPORT(slcd_dma_init);

//����һ��SPI��LCD��DMA�Ĵ���
//x:��ʼ�����ַ���(0~480)
void slcd_dma_enable(u32 x)
{	  
	u32 lcdsize=lcddev.height*lcddev.width;
	u32 dmatransfered=0;
	u32 tick,tick_end;
	u32 time = 0;
	tick = rt_tick_get();
	while(lcdsize)
	{ 
	    time++;
		DMA2_Stream0->CR&=~(1<<0);			//�ر�DMA���� 
		while(DMA2_Stream0->CR&0X01);		//�ȴ�DMA2_Stream0������ 
		DMA2->LIFCR|=1<<5;					//����ϴεĴ�����ɱ��
		if(lcdsize>SLCD_DMA_MAX_TRANS)
		{
			lcdsize-=SLCD_DMA_MAX_TRANS;
			DMA2_Stream0->NDTR=SLCD_DMA_MAX_TRANS;	//���ô��䳤��
		}else
		{
			DMA2_Stream0->NDTR=lcdsize;	//���ô��䳤��
			lcdsize=0;
		}	
		DMA2_Stream0->PAR=(u32)(sramlcdbuf+dmatransfered);	
		dmatransfered+=SLCD_DMA_MAX_TRANS;	
		DMA2_Stream0->CR|=1<<0;				//����DMA���� 		
		//while((DMA2->LISR&(1<<5))==0);		//�ȴ�������� 
		//tick = rt_tick_get();
		rt_sem_take(lcd_dma_int,RT_WAITING_FOREVER);
		//tick_end = rt_tick_get();
		//rt_kprintf("t:%d\n",tick_end-tick);
	} 
	DMA2_Stream0->CR&=~(1<<0);				//�ر�DMA���� 
	tick_end = rt_tick_get();
	//rt_kprintf("dma tick:%d,time:%d\n",tick_end-tick,time);
}
//��ʾһ֡,������һ��spi��lcd����ʾ.
//x:����ƫ����
void slcd_frame_show(void)
{  
	BlockWrite(0,lcddev.width,0,lcddev.height);
	slcd_dma_enable(1);
}
void fill_sram(u16 color)
{
    u32 i;
    for(i=0;i<lcddev.height*lcddev.width;i++)
    {
        sramlcdbuf[i] = color; 
    }
    print_format((u8*)sramlcdbuf,100);
}
void screen_update(void)
{
    u32 i;
    u16 data;
    BlockWrite(0,lcddev.width-1,0,lcddev.height-1);
    for(i=0;i<lcddev.height*lcddev.width;i++)
    {
        data = sramlcdbuf[i];
        //ili9341->ram = data;
        WriteData(data);
    }
}
void screen_color(u16 color)
{
    u32 i;
    LCD_Clear(color);
    return ;
    BlockWrite(0,lcddev.width-1,0,lcddev.height-1);
    for(i=0;i<lcddev.height*lcddev.width;i++)
    {
        ili9341->ram = color;
    }
}

void hello_sram(void)
{
    fill_sram(0);
    slcd_frame_show();
}
void sem_send(void)
{
    rt_sem_release(lcd_dma_int);
}
static void lcd_service_task(void *param)
{
    lcd_dma_int = rt_sem_create("lcd_dma_int",0,RT_IPC_FLAG_FIFO);
    if (lcd_dma_int == RT_NULL)
    {
       // music_log("lcd_dma_int create fail!\n");
        return ;
    }
    while (1)
    {
        rt_sem_take(lcd_dma_int,RT_WAITING_FOREVER);
        slcd_frame_show();
        //rt_thread_delay(10);
        //rt_sem_release(lcd_dma_int);
    }
}
void lcd_dma_startup(void)
{
    rt_thread_t tid;
    
    tid = rt_thread_find("lcd_dma");
    if(tid)
    {
        rt_kprintf("runing this task\n");
        return;
    }
        
    tid = rt_thread_create("lcd_dma",
                           lcd_service_task, 
                           (void *) 0,
                           2048,
                           RT_LCD_THREAD_PRIORITY,
                           20);
    if (tid != RT_NULL)
    {
        rt_thread_startup(tid);
        rt_kprintf("lcd_service_task init done...\n");
    }
    else
    {
        rt_kprintf("lcd_service_task init fail...\n");
    }
}
INIT_APP_EXPORT(lcd_dma_startup);

#ifdef RT_USING_FINSH
#include <finsh.h>
FINSH_FUNCTION_EXPORT(slcd_frame_show, slcd_frame_show);
FINSH_FUNCTION_EXPORT(fill_sram, void fill_sram(u16 color));
FINSH_FUNCTION_EXPORT(screen_update, screen_update);
FINSH_FUNCTION_EXPORT(screen_color, screen_color);
FINSH_FUNCTION_EXPORT(sem_send, sem_send);


#endif





