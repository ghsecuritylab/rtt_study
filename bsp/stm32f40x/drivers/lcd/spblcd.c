#include "spblcd.h"
#include "usart.h"
#include "sram_variable.h"
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


u16 *sramlcdbuf = sramlcdbuf_1;							//SRAM LCD BUFFER,����ͼƬ�Դ��� 

//��ָ��λ�û���.
//x,y:����
//color:��ɫ.�����ң����ϵ���
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
//SRAM --> LCD_RAM dma����
//16λ,�ⲿSRAM���䵽LCD_RAM. 
void slcd_dma_init(void)
{  
	RCC->AHB1ENR|=1<<22;		//DMA2ʱ��ʹ��  
	while(DMA2_Stream0->CR&0X01);//�ȴ�DMA2_Stream0������ 
	DMA2->LIFCR|=0X3D<<6*0;		//���ͨ��0�������жϱ�־
	DMA2_Stream0->FCR=0X0000021;//����ΪĬ��ֵ	
	
	DMA2_Stream0->PAR=0;		//�ݲ�����
	DMA2_Stream0->M0AR=(u32)&LCD->LCD_RAM;	//Ŀ���ַΪLCD_RAM
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
} 
INIT_BOARD_EXPORT(slcd_dma_init);

//����һ��SPI��LCD��DMA�Ĵ���
//x:��ʼ�����ַ���(0~480)
void slcd_dma_enable(u32 x)
{	  
	u32 lcdsize=lcddev.width*lcddev.height;
	u32 dmatransfered=0;
	u32 tick,tick_end;
	tick = rt_tick_get();
	while(lcdsize)
	{ 
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
		lcdsize -= SLCD_DMA_MAX_TRANS;
		DMA2_Stream0->CR|=1<<0;				//����DMA���� 		
		while((DMA2->LISR&(1<<5))==0);		//�ȴ�������� 
	} 
	DMA2_Stream0->CR&=~(1<<0);				//�ر�DMA���� 
	tick_end = rt_tick_get();
	rt_kprintf("dma transfer tick:%d\n",tick_end - tick);
}
//��ʾһ֡,������һ��dma��lcd����ʾ.
//x:����ƫ����
void slcd_frame_show(u32 x)
{  
	#if 0 /*xqy 2020-1-2*/
	//LCD_Scan_Dir(U2D_L2R);		//����ɨ�跽�� �����ң����ϵ��� 
	if(lcddev.id==0X9341||lcddev.id==0X5310||lcddev.id==0X5510||lcddev.id==0X6804)
	{
		LCD_Set_Window(spbdev.stabarheight,0,spbdev.spbheight,spbdev.spbwidth);
		LCD_SetCursor(spbdev.stabarheight,0);//���ù��λ�� 
	}else
	{
		LCD_Set_Window(0,spbdev.stabarheight,spbdev.spbwidth,spbdev.spbheight);
		if(lcddev.id!=0X1963)LCD_SetCursor(0,spbdev.stabarheight);//���ù��λ�� 		
	}
	#endif
	BlockWrite(0,lcddev.width,0,lcddev.height);
	LCD_WriteRAM_Prepare();     //��ʼд��GRAM	
	slcd_dma_enable(x);
	//LCD_Scan_Dir(DFT_SCAN_DIR);	//�ָ�Ĭ�Ϸ���
	//LCD_Set_Window(0,0,lcddev.width,lcddev.height);//�ָ�Ĭ�ϴ��ڴ�С
}
 
void slcd_test(u16 color)
{
    u32 i;
    u32 len = lcddev.height*lcddev.width;
    for(i=0;i++;i<len)
    {   
        sramlcdbuf[i] = color;
    }
    slcd_frame_show(0);
}

#ifdef RT_USING_FINSH

FINSH_FUNCTION_EXPORT(slcd_test, slcd_test dma);
#endif



