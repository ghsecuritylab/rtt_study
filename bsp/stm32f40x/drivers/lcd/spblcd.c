#include "spblcd.h"
#include "usart.h"
#include "sram_variable.h"
#include <rtdef.h>   
#include "drv_lcd.h"  

//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//SPBLCD 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/7/20
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	
extern lcd_info_t lcddev;

//在指定位置画点.
//x,y:坐标
//color:颜色.
void slcd_draw_point(u16 x,u16 y,u16 color)
{	 
	sramlcdbuf[(y-1)*lcddev.width + x -1]=color;
}
//读取指定位置点的颜色值
//x,y:坐标
//返回值:颜色
u16 slcd_read_point(u16 x,u16 y)
{
	return sramlcdbuf[(y-1)*lcddev.width + x -1];
} 
//填充颜色
//x,y:起始坐标
//width，height：宽度和高度
//*color：颜色数组
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
//SRAM --> LCD_RAM dma配置
//16位,外部SRAM传输到LCD_RAM. 
void slcd_dma_init(void)
{  
	RCC->AHB1ENR|=1<<22;		//DMA2时钟使能  
	while(DMA2_Stream0->CR&0X01);//等待DMA2_Stream0可配置 
	DMA2->LIFCR|=0X3D<<6*0;		//清空通道0上所有中断标志
	DMA2_Stream0->FCR=0X0000021;//设置为默认值	
	
	DMA2_Stream0->PAR=0;		//暂不设置
	DMA2_Stream0->M0AR=(u32)&ili9341->ram;	//目标地址为LCD_RAM
	DMA2_Stream0->M1AR=0;		//不用设置
	DMA2_Stream0->NDTR=0;		//暂时设置长度为0
	DMA2_Stream0->CR=0;			//先全部复位CR寄存器值  
	DMA2_Stream0->CR|=2<<6;		//存储器到存储器模式 
	DMA2_Stream0->CR|=0<<8;		//普通模式
	DMA2_Stream0->CR|=1<<9;		//外设增量模式
	DMA2_Stream0->CR|=0<<10;	//存储器非增量模式
	DMA2_Stream0->CR|=1<<11;	//外设数据长度:16位
	DMA2_Stream0->CR|=1<<13;	//存储器数据长度:16位
	DMA2_Stream0->CR|=2<<16;	//高优先级
	DMA2_Stream0->CR|=0<<18;	//单缓冲模式
	DMA2_Stream0->CR|=0<<21;	//外设突发单次传输
	DMA2_Stream0->CR|=0<<23;	//存储器突发单次传输
	DMA2_Stream0->CR|=0<<25;	//选择通道0 
	
	DMA2_Stream0->FCR&=~(1<<2);	//不使用FIFO模式
	DMA2_Stream0->FCR&=~(3<<0);	//无FIFO 设置  
} 
INIT_BOARD_EXPORT(slcd_dma_init);

//开启一次SPI到LCD的DMA的传输
//x:起始传输地址编号(0~480)
void slcd_dma_enable(u32 x)
{	  
	u32 lcdsize=lcddev.height*lcddev.width;
	u32 dmatransfered=0;
	u32 tick,tick_end;
	tick = rt_tick_get();
	while(lcdsize)
	{ 
		DMA2_Stream0->CR&=~(1<<0);			//关闭DMA传输 
		while(DMA2_Stream0->CR&0X01);		//等待DMA2_Stream0可配置 
		DMA2->LIFCR|=1<<5;					//清除上次的传输完成标记
		if(lcdsize>SLCD_DMA_MAX_TRANS)
		{
			lcdsize-=SLCD_DMA_MAX_TRANS;
			DMA2_Stream0->NDTR=SLCD_DMA_MAX_TRANS;	//设置传输长度
		}else
		{
			DMA2_Stream0->NDTR=lcdsize;	//设置传输长度
			lcdsize=0;
		}	
		DMA2_Stream0->PAR=(u32)(sramlcdbuf+dmatransfered);	
		dmatransfered+=SLCD_DMA_MAX_TRANS;	
		DMA2_Stream0->CR|=1<<0;				//开启DMA传输 		
		while((DMA2->LISR&(1<<5))==0);		//等待传输完成 
	} 
	DMA2_Stream0->CR&=~(1<<0);				//关闭DMA传输 
	tick_end = rt_tick_get();
	rt_kprintf("dma tick:%d\n",tick_end-tick);
}
//显示一帧,即启动一次spi到lcd的显示.
//x:坐标偏移量
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

#ifdef RT_USING_FINSH
#include <finsh.h>
FINSH_FUNCTION_EXPORT(slcd_frame_show, slcd_frame_show);
FINSH_FUNCTION_EXPORT(fill_sram, void fill_sram(u16 color));
FINSH_FUNCTION_EXPORT(screen_update, screen_update);
FINSH_FUNCTION_EXPORT(screen_color, screen_color);


#endif





