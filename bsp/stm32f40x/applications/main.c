/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2006-08-31     Bernard      first implementation
 * 2011-06-05     Bernard      modify for STM32F107 version
 */

#include <rthw.h>
#include <rtthread.h>

/**
 * @addtogroup STM32
 */

/*@{*/
#define uart_printf rt_kprintf
void printf_format(char *buf, int len)
{
	int i;
	uart_printf("\r\n");
	for(i=0; i<len; i++)
	{
		if(i>0 && (i%16)==0)
		{
			uart_printf("\r\n");
		}
		uart_printf("%02x ", buf[i]);
	}
	uart_printf("\r\n");
}

char buff[512];
int main(void)
{
    rt_device_t rt_flash_device;
    int offset = 512*100;
    int ret;
    rt_flash_device = rt_device_find("w25qxx");
    if(rt_flash_device == NULL)
        rt_kprintf("没找到设备\r\n");
    test_SD();
    /* user app entry */
    ret = rt_flash_device->open(rt_flash_device,0);
    #if 0
    while(0)
    {
        //rt_kprintf("hello main\r\n");
        //memset(buff,0x8e,512);
        if(rt_flash_device != NULL)
        {
            //ret = W25QXX_Write(buff,offset,30);
            ret = rt_flash_device->write(rt_flash_device,offset,buff,1);
            rt_kprintf("ret1 =%d \r\n",ret);
            memset(buff,0,512);
            ret = rt_flash_device->read(rt_flash_device,offset,buff,1);
            //ret = W25QXX_Read(buff,offset,30);
            rt_kprintf("ret2 =%d \r\n",ret);
            printf_format(buff,30);
        }
        offset+=512;

        rt_thread_delay(200);
    }
    #endif
    return 0;
}

static int read_count = 1024;

void usb_speed_test(void)
{
    int ret;
    int len=0;
    int total_len = 0;
    int tick,tick_1;
    int speed;
    rt_device_t usb;
    unsigned char print_buff[256];
    unsigned char readp[1024];
    usb = rt_device_find("vcom");
    if(!usb)    
    {
        rt_kprintf("cant find vcom\n");
        return ;
    }
    if(rt_device_open(usb,RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX | \
                       RT_DEVICE_FLAG_STREAM) != RT_EOK)
    {
        rt_kprintf("can not open usb\n");
        return ;
    }
    tick = rt_tick_get();
    rt_kprintf("\n");
    while(1)
    {
        ret = rt_device_read(usb,0,readp,read_count);
        if(ret)
        {
            len += ret;
            total_len += ret;
        }

        if(rt_tick_get()-tick>1000)
        {
            tick = rt_tick_get();
            //memset(print_buff,0,256);
            //rt_sprintf(print_buff,"speed:%d ,total:%d                                           \r",len,total_len);
            rt_kprintf("speed:%d ,total:%d\r\n",len,total_len);
            //rt_device_write(usb,0,print_buff,rt_strlen(print_buff));
            len = 0;
        }
    }
}

int change_var(int argc,char *agrv[])
{
    if(argc<2)
    {
        rt_kprintf("err para\n");
        return 0;
    }
    rt_kprintf("read_count:%d\n",agrv[1]);
    read_count = atoi(agrv[1]);
    return 1;
}
void creat_usb_task(void)
{
    rt_thread_t tid;
    tid = rt_thread_create("usb test",usb_speed_test,0,3000,21,10);
    if(tid)
    {
        rt_kprintf("create usb task ok\n");
        rt_thread_startup(tid);
    }
    return ;
}
#ifdef RT_USING_FINSH
#include <finsh.h>
MSH_CMD_EXPORT(creat_usb_task,usb test speed);
MSH_CMD_EXPORT(change_var,set var);

#endif


/*@}*/
