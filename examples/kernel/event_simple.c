/*
 * 程序清单：事件例�?
 *
 * 这个程序会创�?个动态线程及初始化一个静态事件对�?
 * 一个线程等于事件对象上以接收事件；
 * 一个线程定时发送事�?(事件3)
 * 一个线程定时发送事�?(事件5)
 */
#include <rtthread.h>
#include <time.h>
#include "tc_comm.h"

/* 指向线程控制块的指针 */
static rt_thread_t tid1 = RT_NULL;
static rt_thread_t tid2 = RT_NULL;
static rt_thread_t tid3 = RT_NULL;

/* 事件控制�?*/
static struct rt_event event;

/* 线程1入口函数 */
static void thread1_entry(void *param)
{
    rt_uint32_t e;

    while (1)
    {
        /* receive first event */
        if (rt_event_recv(&event, ((1 << 3) | (1 << 5)),
                          RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR,
                          RT_WAITING_FOREVER, &e) == RT_EOK)
        {
            rt_kprintf("thread1: AND recv event 0x%x\n", e);
        }

        rt_kprintf("thread1: delay 1s to prepare second event\n");
        rt_thread_delay(10);

        /* receive second event */
        if (rt_event_recv(&event, ((1 << 3) | (1 << 5)),
                          RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                          RT_WAITING_FOREVER, &e) == RT_EOK)
        {
            rt_kprintf("thread1: OR recv event 0x%x\n", e);
        }

        rt_thread_delay(5);
    }
}

/* 线程2入口函数 */
static void thread2_entry(void *param)
{
    while (1)
    {
        rt_kprintf("thread2: send event1\n");
        rt_event_send(&event, (1 << 3));

        rt_thread_delay(10);
    }
}

/* 线程3入口函数 */
static void thread3_entry(void *param)
{
    while (1)
    {
        rt_kprintf("thread3: send event2\n");
        rt_event_send(&event, (1 << 5));

        rt_thread_delay(20);
    }
}

int event_simple_init()
{
    /* 初始化事件对�?*/
    rt_event_init(&event, "event", RT_IPC_FLAG_FIFO);

    /* 创建线程1 */
    tid1 = rt_thread_create("t1",
                            thread1_entry, RT_NULL, /* 线程入口是thread1_entry, 入口参数是RT_NULL */
                            THREAD_STACK_SIZE, THREAD_PRIORITY, THREAD_TIMESLICE);
    if (tid1 != RT_NULL)
        rt_thread_startup(tid1);
    else
        tc_stat(TC_STAT_END | TC_STAT_FAILED);

    /* 创建线程2 */
    tid2 = rt_thread_create("t2",
                            thread2_entry, RT_NULL, /* 线程入口是thread2_entry, 入口参数是RT_NULL */
                            THREAD_STACK_SIZE, THREAD_PRIORITY, THREAD_TIMESLICE);
    if (tid2 != RT_NULL)
        rt_thread_startup(tid2);
    else
        tc_stat(TC_STAT_END | TC_STAT_FAILED);

    /* 创建线程3 */
    tid3 = rt_thread_create("t3",
                            thread3_entry, RT_NULL, /* 线程入口是thread3_entry, 入口参数是RT_NULL */
                            THREAD_STACK_SIZE, THREAD_PRIORITY, THREAD_TIMESLICE);
    if (tid3 != RT_NULL)
        rt_thread_startup(tid3);
    else
        tc_stat(TC_STAT_END | TC_STAT_FAILED);

    return 0;
}

#ifdef RT_USING_TC
static void _tc_cleanup()
{
    /* 调度器上锁，上锁后，将不再切换到其他线程，仅响应中断 */
    rt_enter_critical();

    /* 删除线程 */
    if (tid1 != RT_NULL && tid1->stat != RT_THREAD_CLOSE)
        rt_thread_delete(tid1);
    if (tid2 != RT_NULL && tid2->stat != RT_THREAD_CLOSE)
        rt_thread_delete(tid2);
    if (tid3 != RT_NULL && tid3->stat != RT_THREAD_CLOSE)
        rt_thread_delete(tid3);

    /* 执行事件对象脱离 */
    rt_event_detach(&event);

    /* 调度器解�?*/
    rt_exit_critical();

    /* 设置TestCase状�?*/
    tc_done(TC_STAT_PASSED);
}

int _tc_event_simple()
{
    /* 设置TestCase清理回调函数 */
    tc_cleanup(_tc_cleanup);
    event_simple_init();

    /* 返回TestCase运行的最长时�?*/
    return 100;
}
/* 输出函数命令到finsh shell�?*/
FINSH_FUNCTION_EXPORT(_tc_event_simple, a simple event example);
#else
/* 用户应用入口 */
int rt_application_init()
{
    event_simple_init();

    return 0;
}
#endif
