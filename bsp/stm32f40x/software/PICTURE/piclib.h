#ifndef __PICLIB_H
#define __PICLIB_H	
#include <rtthread.h>
#include "sys.h" 
#include "malloc.h"
#include "bmp.h"
#include "sram_variable.h"
#include <dfs.h>
#include <dfs_posix.h>

//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//图片解码 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/15
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved
//********************************************************************************
//升级说明
//无
//////////////////////////////////////////////////////////////////////////////////

#define PIC_FORMAT_ERR		0x27	//格式错误
#define PIC_SIZE_ERR		0x28	//图片尺寸错误
#define PIC_WINDOW_ERR		0x29	//窗口设定错误
#define PIC_MEM_ERR			0x11	//内存错误
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef TRUE
#define TRUE    1
#endif
#ifndef FALSE
#define FALSE   0
#endif 
//f_typetell返回的类型定义
//根据表FILE_TYPE_TBL获得.在exfuns.c里面定义
#define T_BIN		0X00	//bin文件
#define T_LRC		0X10	//lrc文件
#define T_NES		0X20	//nes文件

#define T_TEXT		0X30	//.txt文件
#define T_C			0X31	//.c文件
#define T_H			0X32    //.h文件

#define T_WAV		0X40	//WAV文件
#define T_MP3		0X41	//MP3文件 
#define T_APE		0X42	//APE文件
#define T_FLAC		0X43	//FLAC文件

#define T_BMP		0X50	//bmp文件
#define T_JPG		0X51	//jpg文件
#define T_JPEG		0X52	//jpeg文件		 
#define T_GIF		0X53	//gif文件  
 
#define T_AVI		0X60	//avi文件  

#define FILE_MAX_TYPE_NUM		7	//最多FILE_MAX_TYPE_NUM个大类
#define FILE_MAX_SUBT_NUM		4	//最多FILE_MAX_SUBT_NUM个小类

#define _UI_MIN(x, y)           (((x)<(y))?(x):(y))
#define _UI_MAX(x, y)           (((x)>(y))?(x):(y))
#define _UI_BITBYTES(bits)      ((bits + 7)/8)
#define _UI_ABS(x)              ((x)>=0? (x):-(x))

#define RTGUI_RGB_B(c)  ((c) & 0xff)
#define RTGUI_RGB_G(c)  (((c) >> 8)  & 0xff)
#define RTGUI_RGB_R(c)  (((c) >> 16) & 0xff)
#define RTGUI_RGB_A(c)  (((c) >> 24) & 0xff)
/* convert rtgui color to RRRRRGGGGGGBBBBB */
rt_inline rt_uint16_t color_to_565(u32 c)
{
    rt_uint16_t pixel;

    pixel = (rt_uint16_t)(((RTGUI_RGB_R(c) >> 3) << 11) | ((RTGUI_RGB_G(c) >> 2) << 5) | (RTGUI_RGB_B(c) >> 3));

    return pixel;
}


//图片显示物理层接口  
//在移植的时候,必须由用户自己实现这几个函数
typedef struct 
{
	u16(*read_point)(u16,u16);				//u16 read_point(u16 x,u16 y)						读点函数
	void(*draw_point)(u16,u16,u16);			//void draw_point(u16 x,u16 y,u16 color)		    画点函数
 	void(*fill)(u16,u16,u16,u16,u16);		///void fill(u16 sx,u16 sy,u16 ex,u16 ey,u16 color) 单色填充函数 	 
 	void(*draw_hline)(u16,u16,u16,u16);		//void draw_hline(u16 x0,u16 y0,u16 len,u16 color)  画水平线函数	 
 	void(*fillcolor)(u16,u16,u16,u16,u16*);	//void piclib_fill_color(u16 x,u16 y,u16 width,u16 height,u16 *color) 颜色填充
}_pic_phy; 

extern _pic_phy pic_phy;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//图像信息
typedef struct
{		
	u16 lcdwidth;	//LCD的宽度
	u16 lcdheight;	//LCD的高度
	u32 ImgWidth; 	//图像的实际宽度和高度
	u32 ImgHeight;

	u32 Div_Fac;  	//缩放系数 (扩大了8192倍的)
	
	u32 S_Height; 	//设定的高度和宽度
	u32 S_Width;
	
	u32	S_XOFF;	  	//x轴和y轴的偏移量
	u32 S_YOFF;

	u32 staticx; 	//当前显示到的ｘｙ坐标
	u32 staticy;																 	
}_pic_info;
extern _pic_info picinfo;//图像信息

struct rtgui_driver
{
    /* pixel format and byte per pixel */
    rt_uint8_t pixel_format;
    rt_uint8_t bits_per_pixel;
    rt_uint16_t pitch;

    /* screen width and height */
    rt_uint16_t width;
    rt_uint16_t height;

    /* framebuffer address and ops */
    rt_uint8_t *framebuffer;
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
extern void piclib_fill_color(u16 x,u16 y,u16 width,u16 height,u16 *color);
extern void piclib_init(void);								//初始化画图
extern u16 piclib_alpha_blend(u16 src,u16 dst,u8 alpha);	//alphablend处理
extern void ai_draw_init(void);							//初始化智能画图
extern u8 is_element_ok(u16 x,u16 y,u8 chg);				//判断像素是否有效
extern u8 ai_load_picfile(const u8 *filename,u16 x,u16 y,u16 width,u16 height,u8 fast);//智能画图
extern void *pic_memalloc (u32 size);	//pic申请内存
extern void pic_memfree (void* mf);	//pic释放内存

extern u16 get_pixel(u16 x, u16 y);
extern void set_pixel(u16 x, u16 y,u16 c);
extern void piclib_fill(u16 x,u16 y,u16 xe,u16 ye,u16 color);

#define GET_PIXEL(dst, x, y, type)  \
    (type *)((rt_uint8_t*)((dst)->framebuffer) + (y) * (dst)->pitch + (x) * _UI_BITBYTES((dst)->bits_per_pixel))

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#endif






























