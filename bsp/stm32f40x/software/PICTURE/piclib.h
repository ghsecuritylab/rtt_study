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
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//ͼƬ���� ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/15
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved
//********************************************************************************
//����˵��
//��
//////////////////////////////////////////////////////////////////////////////////

#define PIC_FORMAT_ERR		0x27	//��ʽ����
#define PIC_SIZE_ERR		0x28	//ͼƬ�ߴ����
#define PIC_WINDOW_ERR		0x29	//�����趨����
#define PIC_MEM_ERR			0x11	//�ڴ����
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef TRUE
#define TRUE    1
#endif
#ifndef FALSE
#define FALSE   0
#endif 
//f_typetell���ص����Ͷ���
//���ݱ�FILE_TYPE_TBL���.��exfuns.c���涨��
#define T_BIN		0X00	//bin�ļ�
#define T_LRC		0X10	//lrc�ļ�
#define T_NES		0X20	//nes�ļ�

#define T_TEXT		0X30	//.txt�ļ�
#define T_C			0X31	//.c�ļ�
#define T_H			0X32    //.h�ļ�

#define T_WAV		0X40	//WAV�ļ�
#define T_MP3		0X41	//MP3�ļ� 
#define T_APE		0X42	//APE�ļ�
#define T_FLAC		0X43	//FLAC�ļ�

#define T_BMP		0X50	//bmp�ļ�
#define T_JPG		0X51	//jpg�ļ�
#define T_JPEG		0X52	//jpeg�ļ�		 
#define T_GIF		0X53	//gif�ļ�  
 
#define T_AVI		0X60	//avi�ļ�  

#define FILE_MAX_TYPE_NUM		7	//���FILE_MAX_TYPE_NUM������
#define FILE_MAX_SUBT_NUM		4	//���FILE_MAX_SUBT_NUM��С��

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


//ͼƬ��ʾ�����ӿ�  
//����ֲ��ʱ��,�������û��Լ�ʵ���⼸������
typedef struct 
{
	u16(*read_point)(u16,u16);				//u16 read_point(u16 x,u16 y)						���㺯��
	void(*draw_point)(u16,u16,u16);			//void draw_point(u16 x,u16 y,u16 color)		    ���㺯��
 	void(*fill)(u16,u16,u16,u16,u16);		///void fill(u16 sx,u16 sy,u16 ex,u16 ey,u16 color) ��ɫ��亯�� 	 
 	void(*draw_hline)(u16,u16,u16,u16);		//void draw_hline(u16 x0,u16 y0,u16 len,u16 color)  ��ˮƽ�ߺ���	 
 	void(*fillcolor)(u16,u16,u16,u16,u16*);	//void piclib_fill_color(u16 x,u16 y,u16 width,u16 height,u16 *color) ��ɫ���
}_pic_phy; 

extern _pic_phy pic_phy;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//ͼ����Ϣ
typedef struct
{		
	u16 lcdwidth;	//LCD�Ŀ��
	u16 lcdheight;	//LCD�ĸ߶�
	u32 ImgWidth; 	//ͼ���ʵ�ʿ�Ⱥ͸߶�
	u32 ImgHeight;

	u32 Div_Fac;  	//����ϵ�� (������8192����)
	
	u32 S_Height; 	//�趨�ĸ߶ȺͿ��
	u32 S_Width;
	
	u32	S_XOFF;	  	//x���y���ƫ����
	u32 S_YOFF;

	u32 staticx; 	//��ǰ��ʾ���ģ�������
	u32 staticy;																 	
}_pic_info;
extern _pic_info picinfo;//ͼ����Ϣ

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
extern void piclib_init(void);								//��ʼ����ͼ
extern u16 piclib_alpha_blend(u16 src,u16 dst,u8 alpha);	//alphablend����
extern void ai_draw_init(void);							//��ʼ�����ܻ�ͼ
extern u8 is_element_ok(u16 x,u16 y,u8 chg);				//�ж������Ƿ���Ч
extern u8 ai_load_picfile(const u8 *filename,u16 x,u16 y,u16 width,u16 height,u8 fast);//���ܻ�ͼ
extern void *pic_memalloc (u32 size);	//pic�����ڴ�
extern void pic_memfree (void* mf);	//pic�ͷ��ڴ�

extern u16 get_pixel(u16 x, u16 y);
extern void set_pixel(u16 x, u16 y,u16 c);
extern void piclib_fill(u16 x,u16 y,u16 xe,u16 ye,u16 color);

#define GET_PIXEL(dst, x, y, type)  \
    (type *)((rt_uint8_t*)((dst)->framebuffer) + (y) * (dst)->pitch + (x) * _UI_BITBYTES((dst)->bits_per_pixel))

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#endif






























