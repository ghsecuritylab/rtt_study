/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author            Notes
 * 2017-10-30     ZYH            the first version
 */
#include "drv_usbd.h"
#include <rtthread.h>
#include <rtdevice.h>
#include "board.h"
#include "usb_core.h"

#define PRINTF //{rt_kprintf("%s,%d\n",__func__,__LINE__);}
//#define USB_DISCONNECT_PIN                  30        //PA9
static USB_OTG_CORE_HANDLE _stm_pcd;

//static PCD_HandleTypeDef _stm_pcd;
static struct udcd _stm_udc;
static struct ep_id _ep_pool[] =
{
    {0x0,  USB_EP_ATTR_CONTROL,     USB_DIR_INOUT,  64, ID_ASSIGNED  },
    {0x1,  USB_EP_ATTR_BULK,        USB_DIR_IN,     64, ID_UNASSIGNED},
    {0x1,  USB_EP_ATTR_BULK,        USB_DIR_OUT,    64, ID_UNASSIGNED},
    {0x2,  USB_EP_ATTR_INT,         USB_DIR_IN,     64, ID_UNASSIGNED},
    {0x2,  USB_EP_ATTR_INT,         USB_DIR_OUT,    64, ID_UNASSIGNED},
    {0x3,  USB_EP_ATTR_BULK,        USB_DIR_IN,     64, ID_UNASSIGNED},
    {0x3,  USB_EP_ATTR_BULK,        USB_DIR_OUT,    64, ID_UNASSIGNED},
    {0xFF, USB_EP_ATTR_TYPE_MASK,   USB_DIR_MASK,   0,  ID_ASSIGNED  },
};
static int FS_tick = 0;
static int service_ok_tick = 0;
static int count_irq = 0;
static int irq_tick_total = 0;
void OTG_FS_IRQHandler(void)
{
    FS_tick = rt_tick_get();
    count_irq++;
    USB_OTG_DisableGlobalInt(&_stm_pcd);
    /* leave interrupt */
    rt_sem_release(_stm_pcd.isr_sem);

}

void HAL_PCD_ResetCallback(USB_OTG_CORE_HANDLE *pcd)
{
    PRINTF;
    /* open ep0 OUT and IN */
    DCD_EP_Open(pcd, 0x00, 0x40, EP_TYPE_CTRL);
    DCD_EP_Open(pcd, 0x80, 0x40, EP_TYPE_CTRL);
    rt_usbd_reset_handler(&_stm_udc);
}

void HAL_PCD_SetupStageCallback(USB_OTG_CORE_HANDLE *hpcd)
{
    PRINTF;
    rt_usbd_ep0_setup_handler(&_stm_udc, (struct urequest *)hpcd->dev.setup_packet);
}

void HAL_PCD_DataInStageCallback(USB_OTG_CORE_HANDLE *hpcd, uint8_t epnum)
{
    PRINTF;
    if (epnum == 0)
    {
        rt_usbd_ep0_in_handler(&_stm_udc);
    }
    else
    {
        rt_usbd_ep_in_handler(&_stm_udc, 0x80 | epnum, hpcd->dev.in_ep[epnum].xfer_count);
    }
}

void HAL_PCD_ConnectCallback(USB_OTG_CORE_HANDLE *hpcd)
{
    PRINTF;
    rt_usbd_connect_handler(&_stm_udc);
}

void HAL_PCD_SOFCallback(USB_OTG_CORE_HANDLE *hpcd)
{
    //PRINTF;
    rt_usbd_sof_handler(&_stm_udc);
}

void HAL_PCD_DisconnectCallback(USB_OTG_CORE_HANDLE *hpcd)
{
    PRINTF;
    rt_usbd_disconnect_handler(&_stm_udc);
}

void HAL_PCD_DataOutStageCallback(USB_OTG_CORE_HANDLE *hpcd, uint8_t epnum)
{
    PRINTF;
    if (epnum != 0)
    {
        rt_usbd_ep_out_handler(&_stm_udc, epnum, hpcd->dev.out_ep[epnum].xfer_count);
    }
    else
    {
        rt_usbd_ep0_out_handler(&_stm_udc, hpcd->dev.out_ep[0].xfer_count);
    }
}

void HAL_PCDEx_SetConnectionState(USB_OTG_CORE_HANDLE *hpcd, uint8_t state)
{
    PRINTF;
    if (state == 1)
    {
    }
    else
    {
    }
}
static rt_err_t _ep_set_stall(rt_uint8_t address)
{
    if(address)
        DCD_EP_Stall(&_stm_pcd, address);
    else
    {
        DCD_EP_Stall(&_stm_pcd , 0x80);
        DCD_EP_Stall(&_stm_pcd , 0);
        USB_OTG_EP0_OutStart(&_stm_pcd);  
    }
    return RT_EOK;
}

static rt_err_t _ep_clear_stall(rt_uint8_t address)
{
    DCD_EP_ClrStall(&_stm_pcd, address);
    return RT_EOK;
}

static rt_err_t _set_address(rt_uint8_t address)
{
    DCD_EP_SetAddress(&_stm_pcd, address);
    return RT_EOK;
}

static rt_err_t _set_config(rt_uint8_t address)
{
    return RT_EOK;
}

static rt_err_t _ep_enable(uep_t ep)
{
    RT_ASSERT(ep != RT_NULL);
    RT_ASSERT(ep->ep_desc != RT_NULL);
    DCD_EP_Open(&_stm_pcd, ep->ep_desc->bEndpointAddress,
                    ep->ep_desc->wMaxPacketSize, ep->ep_desc->bmAttributes);
    return RT_EOK;
}

static rt_err_t _ep_disable(uep_t ep)
{
    RT_ASSERT(ep != RT_NULL);
    RT_ASSERT(ep->ep_desc != RT_NULL);
    DCD_EP_Close(&_stm_pcd, ep->ep_desc->bEndpointAddress);
    return RT_EOK;
}

static rt_size_t _ep_read(rt_uint8_t address, void *buffer)
{
    rt_size_t size = 0;
    RT_ASSERT(buffer != RT_NULL);
    return size;
}

static rt_size_t _ep_read_prepare(rt_uint8_t address, void *buffer, rt_size_t size)
{
    DCD_EP_PrepareRx(&_stm_pcd, address, buffer, size);
    return size;
}

static rt_size_t _ep_write(rt_uint8_t address, void *buffer, rt_size_t size)
{
    //print_format(buffer,size);
    DCD_EP_Tx(&_stm_pcd, address, buffer, size);
    return size;
}

static rt_err_t _ep0_send_status(void)
{
    //DCD_SetEPStatus(&_stm_pcd, 0x00, 0);
    DCD_EP_Tx (&_stm_pcd,
             0,
             NULL, 
             0); 
  
    USB_OTG_EP0_OutStart(&_stm_pcd);
    return RT_EOK;
}

static rt_err_t _suspend(void)
{
    return RT_EOK;
}

static rt_err_t _wakeup(void)
{
    return RT_EOK;
}

static rt_err_t _init(rt_device_t device)
{
    USB_OTG_BSP_Init(&_stm_pcd); //初始化mcu这边的usb相关硬件环境，GPIO时钟
    
    DCD_Init(&_stm_pcd , 1);//初始化usb模块，模块初始化，设备初始化，还将寄存器地址赋值到_stm_pcd中
    USB_OTG_BSP_EnableInterrupt(&_stm_pcd);//��usb  fs �ж�
    
    return RT_EOK;
}
#define trace_time {int rt_kprintf("tick:d\n",rt_tick_get())};
static void udc_usbd_isr_service(void *param)
{
    while (1)
    {
        rt_sem_take(_stm_pcd.isr_sem, RT_WAITING_FOREVER);
        
        USBD_OTG_ISR_Handler(&_stm_pcd);
        service_ok_tick = rt_tick_get();
        irq_tick_total +=(service_ok_tick - FS_tick);
        //if(count_irq%2000 == 0)
            //rt_kprintf("irq:%d,tick:%d,resent:%d,FS:%d,SER:%d\n",count_irq,irq_tick_total/count_irq,service_ok_tick-FS_tick,FS_tick,service_ok_tick);
        USB_OTG_EnableGlobalInt(&_stm_pcd);
    }
}

const static struct udcd_ops _udc_ops =
{
    _set_address,
    _set_config,
    _ep_set_stall,
    _ep_clear_stall,
    _ep_enable,
    _ep_disable,
    _ep_read_prepare,
    _ep_read,
    _ep_write,
    _ep0_send_status,
    _suspend,
    _wakeup,
};
#define RT_USB_INT_THREAD_PRIORITY 8
int stm_usbd_register(void)
{
    rt_memset((void *)&_stm_udc, 0, sizeof(struct udcd));
    _stm_udc.parent.type = RT_Device_Class_USBDevice;
    _stm_udc.parent.init = _init;
    _stm_udc.parent.user_data = &_stm_pcd;
    _stm_udc.ops = &_udc_ops;
    /* Register endpoint infomation */
    _stm_udc.ep_pool = _ep_pool;
    _stm_udc.ep0.id = &_ep_pool[0];
    rt_device_register((rt_device_t)&_stm_udc, "usbd", 0);
    {
    /* create a ISR service task */
        {
            if (_stm_pcd.isr_sem == RT_NULL)
            {
                _stm_pcd.isr_sem = rt_sem_create("stmSem", 0, RT_IPC_FLAG_FIFO);
                if (!_stm_pcd.isr_sem)
                {
                    rt_kprintf("%s %d sem create err\n", __func__, __LINE__);
                    while (1)
                        ;
                }
            }
            rt_thread_t tid;
    
            tid = rt_thread_create("stmIntSv",
                                   udc_usbd_isr_service, (void *) 0,
                                   2048,
                                   RT_USB_INT_THREAD_PRIORITY,
                                   20);
            if (tid != RT_NULL)
                rt_thread_startup(tid);
            rt_kprintf("STMUSB interrupt service init done...\n");
        }
    }
    rt_usb_device_init();
    return RT_EOK;
}
INIT_DEVICE_EXPORT(stm_usbd_register);



