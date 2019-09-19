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
    rt_kprintf("找到sd0设备\r\n");
    rt_kprintf("找到sd0设备\r\n");
    rt_flash_device = rt_device_find("w25qxx");
    if(rt_flash_device == NULL)
        rt_kprintf("没找到设备\r\n");
    rt_kprintf("找到sd0设备\r\n");
    test_SD();
    /* user app entry */
    ret = rt_flash_device->open(rt_flash_device,0);
    rt_kprintf("ret==%d\r\n",ret);
    while(1)
    {
        //rt_kprintf("hello main\r\n");
        memset(buff,0x8e,512);
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
    return 0;
}

/*@}*/
