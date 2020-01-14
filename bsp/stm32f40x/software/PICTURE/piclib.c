#include "piclib.h"
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
 //文件类型列表
#define RT_PIC_THREAD_PRIORITY 5
u8*const FILE_TYPE_TBL[FILE_MAX_TYPE_NUM][FILE_MAX_SUBT_NUM]=
{
{"BIN"},			//BIN文件
{"LRC"},			//LRC文件
{"NES"},			//NES文件
{"TXT","C","H"},	//文本文件
{"WAV","MP3","APE","FLAC"},//支持的音乐文件
{"BMP","JPG","JPEG","GIF"},//图片文件
{"AVI"},			//视频文件
};

_pic_info picinfo;	 	//图片信息
_pic_phy pic_phy;		//图片显示物理接口	
static struct rtgui_driver _driver;
static struct rtgui_driver *_current_driver = &_driver;
static rt_sem_t pic_sem = RT_NULL;

rt_err_t rtgui_set_device(rt_device_t device)
{
    rt_err_t result;
    struct rt_device_graphic_info info;
    RT_ASSERT(device);
    result = rt_device_open(device, RT_DEVICE_OFLAG_RDWR);
    if (result != RT_EOK)
    {
        return result;
    }

    /* get framebuffer address */
    result = rt_device_control(device, RTGRAPHIC_CTRL_GET_INFO, &info);
    if (result != RT_EOK)
    {
        /* get device information failed */
        return result;
    }
    /* initialize framebuffer driver */
    _driver.pixel_format = info.pixel_format;
    _driver.bits_per_pixel = info.bits_per_pixel;
    _driver.width = info.width;
    _driver.height = info.height;
    _driver.pitch = _driver.width * _UI_BITBYTES(_driver.bits_per_pixel);
    _driver.framebuffer = info.framebuffer;
    rt_kprintf("set lcd ok\n");
    return RT_EOK;
}
void gui_init(void)
{
    rt_device_t device;
    rt_err_t  err;
    
    device = rt_device_find("lcd");
    if(device == RT_NULL)
    {
        rt_kprintf("Not found LCD driver\n");
        return ;
    }
    /* set graphic device */
    err = rtgui_set_device(device);
    piclib_init();
    rt_kprintf("gui_init :%d\n",err);
}
INIT_APP_EXPORT(gui_init);

void set_pixel(u16 x, u16 y,u16 c)
{
    *GET_PIXEL(_current_driver, x, y, rt_uint16_t) = c;
}

u16 get_pixel(u16 x, u16 y)
{
    rt_uint16_t pixel;
    pixel = *GET_PIXEL(_current_driver, x, y, rt_uint16_t);
    return pixel;
}
/* draw raw hline */
void framebuffer_draw_raw_hline(rt_uint8_t *pixels, int x1, int x2, int y)
{
    rt_uint8_t *dst;
    dst = GET_PIXEL(_current_driver, x1, y, rt_uint8_t);
    memcpy(dst, pixels,
           (x2 - x1) * _UI_BITBYTES(_current_driver->bits_per_pixel));
}
void draw_hline(u16 *c, int x1, int x2, int y)
{
    int index;
    rt_uint16_t pixel;
    rt_uint16_t *pixel_ptr;

    /* get pixel from color */
    pixel = *c;//rtgui_color_to_565(*c);

    /* get pixel pointer in framebuffer */
    pixel_ptr = GET_PIXEL(_current_driver, x1, y, rt_uint16_t);

    for (index = x1; index < x2; index ++)
    {
        *pixel_ptr = pixel;
        pixel_ptr ++;
    }
}

void draw_vline(u16 *c, int x , int y1, int y2)
{
    rt_uint8_t *dst;
    rt_uint16_t pixel;
    int index;

    pixel = *c;//rtgui_color_to_565(*c);
    dst = GET_PIXEL(_current_driver, x, y1, rt_uint8_t);
    for (index = y1; index < y2; index ++)
    {
        *(rt_uint16_t *)dst = pixel;
        dst += _current_driver->pitch;
    }
}

//////////////////////////////////////////////////////////////////////////
//lcd.h没有提供划横线函数,需要自己实现
void piclib_draw_hline(u16 x0,u16 y0,u16 len,u16 color)
{
	draw_hline(&color,x0,x0+len,y0);
}
//填充颜色
//x,y:起始坐标
//width，height：宽度和高度。
//*color：颜色数组
void piclib_fill_color(u16 x,u16 y,u16 width,u16 height,u16 *color)
{  
    u32 i;
    for(i=0;i<height;i++)
    {
        draw_hline(color,x,x+width,y+i);
    }
}
void piclib_fill(u16 x,u16 y,u16 xe,u16 ye,u16 color)
{  
    u32 i;
    u32 height = ye - y + 1;
    for(i=0;i<height;i++)
    {
        draw_hline(&color,x,xe,y+i);
    }
}

//将小写字母转为大写字母,如果是数字,则保持不变.
u8 char_upper(u8 c)
{
	if(c<'A')return c;//数字,保持不变.
	if(c>='a')return c-0x20;//变为大写.
	else return c;//大写,保持不变
}

u8 f_typetell(u8 *fname)
{
	u8 tbuf[5];
	u8 *attr='\0';//后缀名
	u8 i=0,j;
	while(i<250)
	{
		i++;
		if(*fname=='\0')break;//偏移到了最后了.
		fname++;
	}
	if(i==250)return 0XFF;//错误的字符串.
 	for(i=0;i<5;i++)//得到后缀名
	{
		fname--;
		if(*fname=='.')
		{
			fname++;
			attr=fname;
			break;
		}
  	}
	strcpy((char *)tbuf,(const char*)attr);//copy
 	for(i=0;i<4;i++)tbuf[i]=char_upper(tbuf[i]);//全部变为大写 
	for(i=0;i<FILE_MAX_TYPE_NUM;i++)	//大类对比
	{
		for(j=0;j<FILE_MAX_SUBT_NUM;j++)//子类对比
		{
			if(*FILE_TYPE_TBL[i][j]==0)break;//此组已经没有可对比的成员了.
			if(strcmp((const char *)FILE_TYPE_TBL[i][j],(const char *)tbuf)==0)//找到了
			{
				return (i<<4)|j;
			}
		}
	}
	return 0XFF;//没找到		 			   
}	 

//////////////////////////////////////////////////////////////////////////
//画图初始化,在画图之前,必须先调用此函数
//指定画点/读点
void piclib_init(void)
{
	pic_phy.read_point=get_pixel;  		//读点函数实现,仅BMP需要
	pic_phy.draw_point=set_pixel;	//画点函数实现
	pic_phy.fill=piclib_fill;					//填充函数实现,仅GIF需要
	pic_phy.draw_hline=piclib_draw_hline;  	//画线函数实现,仅GIF需要
	pic_phy.fillcolor=piclib_fill_color;  	//颜色填充函数实现,仅TJPGD需要 

	picinfo.lcdwidth=_driver.width;	//得到LCD的宽度像素
	picinfo.lcdheight=_driver.height;//得到LCD的高度像素

	picinfo.ImgWidth=0;	//初始化宽度为0
	picinfo.ImgHeight=0;//初始化高度为0
	picinfo.Div_Fac=0;	//初始化缩放系数为0
	picinfo.S_Height=0;	//初始化设定的高度为0
	picinfo.S_Width=0;	//初始化设定的宽度为0
	picinfo.S_XOFF=0;	//初始化x轴的偏移量为0
	picinfo.S_YOFF=0;	//初始化y轴的偏移量为0
	picinfo.staticx=0;	//初始化当前显示到的x坐标为0
	picinfo.staticy=0;	//初始化当前显示到的y坐标为0
}
//快速ALPHA BLENDING算法.
//src:源颜色
//dst:目标颜色
//alpha:透明程度(0~32)
//返回值:混合后的颜色.
u16 piclib_alpha_blend(u16 src,u16 dst,u8 alpha)
{
	u32 src2;
	u32 dst2;	 
	//Convert to 32bit |-----GGGGGG-----RRRRR------BBBBB|
	src2=((src<<16)|src)&0x07E0F81F;
	dst2=((dst<<16)|dst)&0x07E0F81F;   
	//Perform blending R:G:B with alpha in range 0..32
	//Note that the reason that alpha may not exceed 32 is that there are only
	//5bits of space between each R:G:B value, any higher value will overflow
	//into the next component and deliver ugly result.
	dst2=((((dst2-src2)*alpha)>>5)+src2)&0x07E0F81F;
	return (dst2>>16)|dst2;  
}
//初始化智能画点
//内部调用
void ai_draw_init(void)
{
	float temp,temp1;	   
	temp=(float)picinfo.S_Width/picinfo.ImgWidth;
	temp1=(float)picinfo.S_Height/picinfo.ImgHeight;						 
	if(temp<temp1)temp1=temp;//取较小的那个	 
	if(temp1>1)temp1=1;	  
	//使图片处于所给区域的中间
	picinfo.S_XOFF+=(picinfo.S_Width-temp1*picinfo.ImgWidth)/2;
	picinfo.S_YOFF+=(picinfo.S_Height-temp1*picinfo.ImgHeight)/2;
	temp1*=8192;//扩大8192倍	 
	picinfo.Div_Fac=temp1;
	picinfo.staticx=0xffff;
	picinfo.staticy=0xffff;//放到一个不可能的值上面			 										    
}   
//判断这个像素是否可以显示
//(x,y) :像素原始坐标
//chg   :功能变量. 
//返回值:0,不需要显示.1,需要显示
u8 is_element_ok(u16 x,u16 y,u8 chg)
{				  
	if(x!=picinfo.staticx||y!=picinfo.staticy)
	{
		if(chg==1)
		{
			picinfo.staticx=x;
			picinfo.staticy=y;
		} 
		return 1;
	}else return 0;
}
//智能画图
//FileName:要显示的图片文件  BMP/JPG/JPEG/GIF
//x,y,width,height:坐标及显示区域尺寸
//fast:使能jpeg/jpg小图片(图片尺寸小于等于液晶分辨率)快速解码,0,不使能;1,使能.
//图片在开始和结束的坐标点范围内显示
u8 ai_load_picfile(const u8 *filename,u16 x,u16 y,u16 width,u16 height,u8 fast)
{	
	u8	res;//返回值
	u8 temp;	
	u32 tick,tick_end;
	if((x+width)>picinfo.lcdwidth)
	    return PIC_WINDOW_ERR;		//x坐标超范围了.
	if((y+height)>picinfo.lcdheight)
	    return PIC_WINDOW_ERR;		//y坐标超范围了.  
	//得到显示方框大小	  	 
	if(width==0||height==0)
	    return PIC_WINDOW_ERR;	//窗口设定错误
	picinfo.S_Height=height;
	picinfo.S_Width=width;
	//显示区域无效
	if(picinfo.S_Height==0||picinfo.S_Width==0)
	{
		picinfo.S_Height=_current_driver->height;
		picinfo.S_Width=_current_driver->width;
		return FALSE;   
	}
	if(pic_phy.fillcolor==NULL)
	    fast=0;//颜色填充函数未实现,不能快速显示
	//显示的开始坐标点
	picinfo.S_YOFF=y;
	picinfo.S_XOFF=x;
	//文件名传递		 
	temp=f_typetell((u8*)filename);	//得到文件的类型
	tick = rt_tick_get();
	switch(temp)
	{											  
		case T_BMP:
			res=stdbmp_decode(filename); 				//解码bmp	  	  
			break;
		case T_JPG:
		case T_JPEG:
			//res=jpg_decode(filename,fast);				//解码JPG/JPEG	  	  
			break;
		case T_GIF:
			//res=gif_decode(filename,x,y,width,height);	//解码gif  	  
			break;
		default:
	 		res=PIC_FORMAT_ERR;  						//非图片格式!!!  
			break;
	}  
	tick_end = rt_tick_get();
	rt_kprintf("tick1:%d\n",tick_end-tick);
	return res;
}
//动态分配内存
void *pic_memalloc (u32 size)			
{
	return (void*)mymalloc(1,size);
}
//释放内存
void pic_memfree (void* mf)		 
{
	myfree(1,mf);
}
int pic_auto_play(char * file_dir_name)
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
    rt_memset(temp_file_name,0,60);
    rt_strncpy(temp_file_name,file_dir_name,rt_strlen(file_dir_name));
    rt_strncpy(temp_file_name+rt_strlen(temp_file_name),file_info->d_name,rt_strlen(file_info->d_name));
    //rt_kprintf("当前图片为=%s\n",temp_file_name);
    ai_load_picfile(temp_file_name,0,0,_current_driver->width,_current_driver->height,0);
    return 0;
}
static void pic_service_task(void *param)
{
    pic_sem = rt_sem_create("pic_sem",0,RT_IPC_FLAG_FIFO);
    if (pic_sem == RT_NULL)
    {
        rt_kprintf("pic_sem create fail!\n");
        return ;
    }
    while (1)
    {
        rt_sem_take(pic_sem,RT_WAITING_FOREVER);
        pic_auto_play("/picture/");
        screen_update();
        screen_update();
        rt_thread_delay(1500);
        rt_sem_release(pic_sem);
    }
}
void pic_task_startup(void)
{
    rt_thread_t tid;
    
    tid = rt_thread_find("pic_task");
    if(tid)
    {
        rt_kprintf("runing this task\n");
        return;
    }
        
    tid = rt_thread_create("pic_task",
                           pic_service_task, 
                           (void *) 0,
                           1024,
                           RT_PIC_THREAD_PRIORITY,
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
INIT_APP_EXPORT(pic_task_startup);

void pic(void)
{
    if(!pic_sem)
    {
        rt_kprintf("please init pic_sem first\n");
        return ;
    }
    rt_sem_release(pic_sem);
}
#ifdef RT_USING_FINSH
#include <finsh.h>
MSH_CMD_EXPORT(pic, ----pic----);


#endif
























