/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2006-03-17     Bernard      the first version
 * 2006-04-28     Bernard      fix the scheduler algorthm
 * 2006-04-30     Bernard      add SCHEDULER_DEBUG
 * 2006-05-27     Bernard      fix the scheduler algorthm for same priority
 *                             thread schedule
 * 2006-06-04     Bernard      rewrite the scheduler algorithm
 * 2006-08-03     Bernard      add hook support
 * 2006-09-05     Bernard      add 32 priority level support
 * 2006-09-24     Bernard      add rt_system_scheduler_start function
 * 2009-09-16     Bernard      fix _rt_scheduler_stack_check
 * 2010-04-11     yi.qiu       add module feature
 * 2010-07-13     Bernard      fix the maximal number of rt_scheduler_lock_nest
 *                             issue found by kuronca
 * 2010-12-13     Bernard      add defunct list initialization even if not use heap.
 * 2011-05-10     Bernard      clean scheduler debug log.
 * 2013-12-21     Grissiom     add rt_critical_level
 * 2018-11-22     Jesven       remove the current task from ready queue
 *                             add per cpu ready queue
 *                             add _get_highest_priority_thread to find highest priority task
 *                             rt_schedule_insert_thread won't insert current task to ready queue
 *                             in smp version, rt_hw_context_switch_interrupt maybe switch to
 *                               new task directly
 *
 */

#include <rtthread.h>
#include <rthw.h>

#ifdef RT_USING_SMP
rt_hw_spinlock_t _rt_critical_lock;
#endif /*RT_USING_SMP*/

rt_list_t rt_thread_priority_table[RT_THREAD_PRIORITY_MAX];//优先级线程链表数组，每个优先级一个入口节点
                                                           //每个节点表示对应优先级的任务的入口，相同优先级
                                                           //任务插入在链表最后，时间片轮转
rt_uint32_t rt_thread_ready_priority_group;//32位数据，每一位代表一个优先级，0代表未就绪，1代表就绪
#if RT_THREAD_PRIORITY_MAX > 32
/* Maximum priority level, 256 */
rt_uint8_t rt_thread_ready_table[32];//最大支持256任务，通过32字节来表示，将该数组看成一个整体，每位代表一个级
//32字节刚好是256位
#endif

#ifndef RT_USING_SMP
extern volatile rt_uint8_t rt_interrupt_nest;//中断嵌套变量
static rt_int16_t rt_scheduler_lock_nest;//调度锁嵌套变量
struct rt_thread *rt_current_thread;//当前线程
rt_uint8_t rt_current_priority;//当前优先级
#endif /*RT_USING_SMP*/

rt_list_t rt_thread_defunct;

#ifdef RT_USING_HOOK
static void (*rt_scheduler_hook)(struct rt_thread *from, struct rt_thread *to);

/**
 * @addtogroup Hook
 */

/**@{*/

/**
 * This function will set a hook function, which will be invoked when thread
 * switch happens.
 *
 * @param hook the hook function
 */
void
rt_scheduler_sethook(void (*hook)(struct rt_thread *from, struct rt_thread *to))
{
    rt_scheduler_hook = hook;
}

/**@}*/
#endif

#ifdef RT_USING_OVERFLOW_CHECK
static void _rt_scheduler_stack_check(struct rt_thread *thread)
{
    RT_ASSERT(thread != RT_NULL);

#if defined(ARCH_CPU_STACK_GROWS_UPWARD)
	if (*((rt_uint8_t *)((rt_ubase_t)thread->stack_addr + thread->stack_size - 1)) != '#' ||
#else
    if (*((rt_uint8_t *)thread->stack_addr) != '#' ||
#endif
        (rt_ubase_t)thread->sp <= (rt_ubase_t)thread->stack_addr ||
        (rt_ubase_t)thread->sp >
        (rt_ubase_t)thread->stack_addr + (rt_ubase_t)thread->stack_size)
    {
        rt_ubase_t level;

        rt_kprintf("thread:%s stack overflow\n", thread->name);
#ifdef RT_USING_FINSH
        {
            extern long list_thread(void);
            list_thread();
        }
#endif
        level = rt_hw_interrupt_disable();
        while (level);
    }
#if defined(ARCH_CPU_STACK_GROWS_UPWARD)
    else if ((rt_ubase_t)thread->sp > ((rt_ubase_t)thread->stack_addr + thread->stack_size))
    {
        rt_kprintf("warning: %s stack is close to the top of stack address.\n",
                   thread->name);
    }
#else
    else if ((rt_ubase_t)thread->sp <= ((rt_ubase_t)thread->stack_addr + 32))
    {
        rt_kprintf("warning: %s stack is close to end of stack address.\n",
                   thread->name);
    }
#endif
}
#endif

/*
 * get the highest priority thread in ready queue
 */
#ifdef RT_USING_SMP
static struct rt_thread* _get_highest_priority_thread(rt_ubase_t *highest_prio)
{
    register struct rt_thread *highest_priority_thread;
    register rt_ubase_t highest_ready_priority, local_highest_ready_priority;
    struct rt_cpu* pcpu = rt_cpu_self();

#if RT_THREAD_PRIORITY_MAX > 32
    register rt_ubase_t number;

    if (rt_thread_ready_priority_group == 0 && pcpu->priority_group == 0)
    {
        *highest_prio = pcpu->current_thread->current_priority;
        /* only local IDLE is readly */
        return pcpu->current_thread;
    }

    number = __rt_ffs(rt_thread_ready_priority_group) - 1;
    highest_ready_priority = (number << 3) + __rt_ffs(rt_thread_ready_table[number]) - 1;
    number = __rt_ffs(pcpu->priority_group) - 1;
    local_highest_ready_priority = (number << 3) + __rt_ffs(pcpu->ready_table[number]) - 1;
#else
    highest_ready_priority = __rt_ffs(rt_thread_ready_priority_group) - 1;
    local_highest_ready_priority = __rt_ffs(pcpu->priority_group) - 1;
#endif

    /* get highest ready priority thread */
    if (highest_ready_priority < local_highest_ready_priority)
    {
        *highest_prio = highest_ready_priority;
        highest_priority_thread = rt_list_entry(rt_thread_priority_table[highest_ready_priority].next,
                                  struct rt_thread,
                                  tlist);
    }
    else
    {
        *highest_prio = local_highest_ready_priority;
        highest_priority_thread = rt_list_entry(pcpu->priority_table[local_highest_ready_priority].next,
                                  struct rt_thread,
                                  tlist);
    }

    return highest_priority_thread;
}
#else
static struct rt_thread* _get_highest_priority_thread(rt_ubase_t *highest_prio)
{
    register struct rt_thread *highest_priority_thread;
    register rt_ubase_t highest_ready_priority;

#if RT_THREAD_PRIORITY_MAX > 32
    register rt_ubase_t number;

    number = __rt_ffs(rt_thread_ready_priority_group) - 1;//现在所在数组，number乘以8，获得该数组的起始优先级
    highest_ready_priority = (number << 3) + __rt_ffs(rt_thread_ready_table[number]) - 1;//起始优先级加上数组中的序号
    //就等于实际的优先级
#else
    highest_ready_priority = __rt_ffs(rt_thread_ready_priority_group) - 1;//32优先级使用，为啥加减一待研究
#endif

    /* get highest ready priority thread */
    //一个就绪线程链表中，最先执行的为链表入口节点的下一个节点，运行完后将其插入到最后
    //该函数通过结构体中的成员地址反推出该结构体的首地址，而获取到完成的线程控制块
    highest_priority_thread = rt_list_entry(rt_thread_priority_table[highest_ready_priority].next,
                              struct rt_thread,
                              tlist);

    *highest_prio = highest_ready_priority;

    return highest_priority_thread;
}
#endif

/**
 * @ingroup SystemInit
 * This function will initialize the system scheduler
 */
void rt_system_scheduler_init(void)
{
#ifdef RT_USING_SMP
    int cpu;
#endif /*RT_USING_SMP*/
    register rt_base_t offset;

#ifndef RT_USING_SMP
    rt_scheduler_lock_nest = 0;
#endif /*RT_USING_SMP*/

    RT_DEBUG_LOG(RT_DEBUG_SCHEDULER, ("start scheduler: max priority 0x%02x\n",
                                      RT_THREAD_PRIORITY_MAX));

    for (offset = 0; offset < RT_THREAD_PRIORITY_MAX; offset ++)
    {
        rt_list_init(&rt_thread_priority_table[offset]);//双向链表的初始化，前后指针都指向自己
    }
#ifdef RT_USING_SMP
    for (cpu = 0; cpu < RT_CPUS_NR; cpu++)
    {
        struct rt_cpu *pcpu =  rt_cpu_index(cpu);
        for (offset = 0; offset < RT_THREAD_PRIORITY_MAX; offset ++)
        {
            rt_list_init(&pcpu->priority_table[offset]);
        }

        pcpu->irq_switch_flag = 0;
        pcpu->current_priority = RT_THREAD_PRIORITY_MAX - 1;
        pcpu->current_thread = RT_NULL;
        pcpu->priority_group = 0;

#if RT_THREAD_PRIORITY_MAX > 32
        rt_memset(pcpu->ready_table, 0, sizeof(pcpu->ready_table));
#endif
    }
#endif /*RT_USING_SMP*/

    /* initialize ready priority group */
    rt_thread_ready_priority_group = 0;//初始化为所有都未就绪

#if RT_THREAD_PRIORITY_MAX > 32
    /* initialize ready table */
    rt_memset(rt_thread_ready_table, 0, sizeof(rt_thread_ready_table));//255任务所需要
#endif

    /* initialize thread defunct */
    rt_list_init(&rt_thread_defunct);//默认线程链表入口节点，当有线程delete，detach，exit时会将其插入到最后
}

/**
 * @ingroup SystemInit
 * This function will startup scheduler. It will select one thread
 * with the highest priority level, then switch to it.
 */
void rt_system_scheduler_start(void)
{
    register struct rt_thread *to_thread;
    rt_ubase_t highest_ready_priority;

    to_thread = _get_highest_priority_thread(&highest_ready_priority);//查找最高优先级和对应任务

#ifdef RT_USING_SMP
    to_thread->oncpu = rt_hw_cpu_id();
#else
    rt_current_thread = to_thread;
#endif /*RT_USING_SMP*/

    rt_schedule_remove_thread(to_thread);//将线程溢出就绪链表
    to_thread->stat = RT_THREAD_RUNNING;

    /* switch to new thread */
#ifdef RT_USING_SMP
    rt_hw_context_switch_to((rt_ubase_t)&to_thread->sp, to_thread);
#else
    rt_hw_context_switch_to((rt_ubase_t)&to_thread->sp);//压栈处理，切换到任务开始运行
#endif /*RT_USING_SMP*/

    /* never come back */
}

/**
 * @addtogroup Thread
 */

/**@{*/


#ifdef RT_USING_SMP
/**
 * This function will handle IPI interrupt and do a scheduling in system;
 * 
 * @param vector, the number of IPI interrupt for system scheduling
 * @param param, use RT_NULL
 * 
 * NOTE: this function should be invoke or register as ISR in BSP.
 */
void rt_scheduler_ipi_handler(int vector, void *param)
{
    rt_schedule();
}

/**
 * This function will perform one scheduling. It will select one thread
 * with the highest priority level in global ready queue or local ready queue, 
 * then switch to it.
 */
void rt_schedule(void)
{
    rt_base_t level;
    struct rt_thread *to_thread;
    struct rt_thread *current_thread;
    struct rt_cpu    *pcpu;
    int cpu_id;

    /* disable interrupt */
    level  = rt_hw_interrupt_disable();

    cpu_id = rt_hw_cpu_id();
    pcpu   = rt_cpu_index(cpu_id);
    current_thread = pcpu->current_thread;

    /* whether do switch in interrupt */
    if (pcpu->irq_nest)
    {
        pcpu->irq_switch_flag = 1;
    }
    else if (current_thread->scheduler_lock_nest == 1) /* whether lock scheduler */
    {
        rt_ubase_t highest_ready_priority;

        if (rt_thread_ready_priority_group != 0 || pcpu->priority_group != 0)
        {
            to_thread = _get_highest_priority_thread(&highest_ready_priority);
            current_thread->oncpu = RT_CPU_DETACHED;
            if ((current_thread->stat & RT_THREAD_STAT_MASK) == RT_THREAD_RUNNING)
            {
                if (current_thread->current_priority < highest_ready_priority)
                {
                    to_thread = current_thread;
                }
                else
                {
                    rt_schedule_insert_thread(current_thread);
                }
            }
            to_thread->oncpu = cpu_id;
            if (to_thread != current_thread)
            {
                /* if the destination thread is not the same as current thread */
                pcpu->current_priority = (rt_uint8_t)highest_ready_priority;

                RT_OBJECT_HOOK_CALL(rt_scheduler_hook, (current_thread, to_thread));

                rt_schedule_remove_thread(to_thread);
                to_thread->stat = RT_THREAD_RUNNING | (to_thread->stat & ~RT_THREAD_STAT_MASK);

                /* switch to new thread */
                RT_DEBUG_LOG(RT_DEBUG_SCHEDULER,
                        ("[%d]switch to priority#%d "
                         "thread:%.*s(sp:0x%08x), "
                         "from thread:%.*s(sp: 0x%08x)\n",
                         pcpu->irq_nest, highest_ready_priority,
                         RT_NAME_MAX, to_thread->name, to_thread->sp,
                         RT_NAME_MAX, current_thread->name, current_thread->sp));

#ifdef RT_USING_OVERFLOW_CHECK
                _rt_scheduler_stack_check(to_thread);
#endif

                {
                    extern void rt_thread_handle_sig(rt_bool_t clean_state);

                    rt_hw_context_switch((rt_ubase_t)&current_thread->sp,
                                         (rt_ubase_t)&to_thread->sp, to_thread);

                    /* enable interrupt */
                    rt_hw_interrupt_enable(level);

#ifdef RT_USING_SIGNALS
                    /* check signal status */
                    rt_thread_handle_sig(RT_TRUE);
#endif
                    goto __exit;
                }
            }
        }
    }

    /* enable interrupt */
    rt_hw_interrupt_enable(level);

__exit:
    return ;
}
#else
/**
 * This function will perform one schedule. It will select one thread
 * with the highest priority level, then switch to it.
 */
void rt_schedule(void)
{
    rt_base_t level;
    struct rt_thread *to_thread;
    struct rt_thread *from_thread;

    /* disable interrupt */
    level = rt_hw_interrupt_disable();

    /* check the scheduler is enabled or not */
    if (rt_scheduler_lock_nest == 0)//调度器没锁则调度，不为0代表被锁，不调度任务
    {
        rt_ubase_t highest_ready_priority;

        if (rt_thread_ready_priority_group != 0)//如果为0会怎么样的，为啥会怎么判断呢？
        {
            /* need_insert_from_thread: need to insert from_thread to ready queue */
            int need_insert_from_thread = 0;

            to_thread = _get_highest_priority_thread(&highest_ready_priority);//该获取的线程肯定不是当前线程rt_current_thread
            //因为当前运行的线程是被移出了就绪表的
            if ((rt_current_thread->stat & RT_THREAD_STAT_MASK) == RT_THREAD_RUNNING)
            {
                if (rt_current_thread->current_priority < highest_ready_priority)//让出后发现没有其他相同优先级线程
                {
                    to_thread = rt_current_thread;//这个可能出现在线程让出控制权给相同优先级线程时
                }
                else
                {
                    //该步出现在：
                    //1:当前优先级大于就绪优先级：当前线程被抢占
                    //2:当前优先级等于就绪优先级：当前线程时间片到时了，让出CPU给相同优先级的其他任务
                    //需要将该线程加入对应优先级线程链表的尾部
                    need_insert_from_thread = 1;
                }
            }

            if (to_thread != rt_current_thread)
            {
                /* if the destination thread is not the same as current thread */
                rt_current_priority = (rt_uint8_t)highest_ready_priority;
                from_thread         = rt_current_thread;
                rt_current_thread   = to_thread;

                RT_OBJECT_HOOK_CALL(rt_scheduler_hook, (from_thread, to_thread));

                if (need_insert_from_thread)
                {
                    rt_schedule_insert_thread(from_thread);//将线程插入就绪列表尾部
                    //那被抢占的任务也被放在尾部了？等于放弃了剩余的时间片了？
                }

                rt_schedule_remove_thread(to_thread);//将将要运行的任务移出链表
                to_thread->stat = RT_THREAD_RUNNING | (to_thread->stat & ~RT_THREAD_STAT_MASK);//改为运行态

                /* switch to new thread */
                RT_DEBUG_LOG(RT_DEBUG_SCHEDULER,
                        ("[%d]switch to priority#%d "
                         "thread:%.*s(sp:0x%08x), "
                         "from thread:%.*s(sp: 0x%08x)\n",
                         rt_interrupt_nest, highest_ready_priority,
                         RT_NAME_MAX, to_thread->name, to_thread->sp,
                         RT_NAME_MAX, from_thread->name, from_thread->sp));

#ifdef RT_USING_OVERFLOW_CHECK
                _rt_scheduler_stack_check(to_thread);
#endif

                if (rt_interrupt_nest == 0)//线程调度
                {
                    extern void rt_thread_handle_sig(rt_bool_t clean_state);

                    rt_hw_context_switch((rt_ubase_t)&from_thread->sp,
                            (rt_ubase_t)&to_thread->sp);

                    /* enable interrupt */
                    rt_hw_interrupt_enable(level);

#ifdef RT_USING_SIGNALS
                    /* check signal status */
                    rt_thread_handle_sig(RT_TRUE);
#endif
                    goto __exit;
                }
                else//中断调度
                {
                    RT_DEBUG_LOG(RT_DEBUG_SCHEDULER, ("switch in interrupt\n"));

                    rt_hw_context_switch_interrupt((rt_ubase_t)&from_thread->sp,
                            (rt_ubase_t)&to_thread->sp);
                }
            }
            else
            {
                //该种情况只存在：当前线程rt_thread_yield时，但无其他同级任务，又重新开始运行当前的任务
                rt_schedule_remove_thread(rt_current_thread);//将当前的任务移出就绪链表
                //但为啥需要移除该链表呢，是在什么情况下任务还在就绪列表中呢？该链表是正在运行的，是不存在就绪链表当中的
                rt_current_thread->stat = RT_THREAD_RUNNING | (rt_current_thread->stat & ~RT_THREAD_STAT_MASK);
                //将线程状态修改为正在运行态
            }
        }
    }

    /* enable interrupt */
    rt_hw_interrupt_enable(level);

__exit:
    return;
}
#endif /*RT_USING_SMP*/

/**
 * This function checks if a scheduling is needed after IRQ context. If yes,
 * it will select one thread with the highest priority level, and then switch
 * to it.
 */
#ifdef RT_USING_SMP
void rt_scheduler_do_irq_switch(void *context)
{
    int cpu_id;
    rt_base_t level;
    struct rt_cpu* pcpu;
    struct rt_thread *to_thread;
    struct rt_thread *current_thread;

    level = rt_hw_interrupt_disable();

    cpu_id = rt_hw_cpu_id();
    pcpu   = rt_cpu_index(cpu_id);
    current_thread = pcpu->current_thread;

    if (pcpu->irq_switch_flag == 0)
    {
        rt_hw_interrupt_enable(level);
        return;
    }

    if (current_thread->scheduler_lock_nest == 1 && pcpu->irq_nest == 0)
    {
        rt_ubase_t highest_ready_priority;

        /* clear irq switch flag */
        pcpu->irq_switch_flag = 0;

        if (rt_thread_ready_priority_group != 0 || pcpu->priority_group != 0)
        {
            to_thread = _get_highest_priority_thread(&highest_ready_priority);
            current_thread->oncpu = RT_CPU_DETACHED;
            if ((current_thread->stat & RT_THREAD_STAT_MASK) == RT_THREAD_RUNNING)
            {
                if (current_thread->current_priority < highest_ready_priority)
                {
                    to_thread = current_thread;
                }
                else
                {
                    rt_schedule_insert_thread(current_thread);
                }
            }
            to_thread->oncpu = cpu_id;
            if (to_thread != current_thread)
            {
                /* if the destination thread is not the same as current thread */

                pcpu->current_priority = (rt_uint8_t)highest_ready_priority;

                RT_OBJECT_HOOK_CALL(rt_scheduler_hook, (current_thread, to_thread));

                rt_schedule_remove_thread(to_thread);
                to_thread->stat = RT_THREAD_RUNNING | (to_thread->stat & ~RT_THREAD_STAT_MASK);

#ifdef RT_USING_OVERFLOW_CHECK
                _rt_scheduler_stack_check(to_thread);
#endif
                RT_DEBUG_LOG(RT_DEBUG_SCHEDULER, ("switch in interrupt\n"));

                current_thread->cpus_lock_nest--;
                current_thread->scheduler_lock_nest--;

                rt_hw_context_switch_interrupt(context, (rt_ubase_t)&current_thread->sp,
                        (rt_ubase_t)&to_thread->sp, to_thread);
            }
        }
    }
    rt_hw_interrupt_enable(level);
}
#endif /*RT_USING_SMP*/

/*
 * This function will insert a thread to system ready queue. The state of
 * thread will be set as READY and remove from suspend queue.
 *
 * @param thread the thread to be inserted
 * @note Please do not invoke this function in user application.
 */
#ifdef RT_USING_SMP
void rt_schedule_insert_thread(struct rt_thread *thread)
{
    int cpu_id;
    int bind_cpu;
    rt_uint32_t cpu_mask;
    register rt_base_t level;

    RT_ASSERT(thread != RT_NULL);

    /* disable interrupt */
    level = rt_hw_interrupt_disable();

    /* it should be RUNNING thread */
    if (thread->oncpu != RT_CPU_DETACHED)
    {
        thread->stat = RT_THREAD_RUNNING | (thread->stat & ~RT_THREAD_STAT_MASK);
        goto __exit;
    }

    /* READY thread, insert to ready queue */
    thread->stat = RT_THREAD_READY | (thread->stat & ~RT_THREAD_STAT_MASK);

    cpu_id   = rt_hw_cpu_id();
    bind_cpu = thread->bind_cpu ;

    /* insert thread to ready list */
    if (bind_cpu == RT_CPUS_NR)
    {
#if RT_THREAD_PRIORITY_MAX > 32
        rt_thread_ready_table[thread->number] |= thread->high_mask;
#endif
        rt_thread_ready_priority_group |= thread->number_mask;

        rt_list_insert_before(&(rt_thread_priority_table[thread->current_priority]),
                              &(thread->tlist));
        cpu_mask = RT_CPU_MASK ^ (1 << cpu_id);
        rt_hw_ipi_send(RT_SCHEDULE_IPI, cpu_mask);
    }
    else
    {
        struct rt_cpu *pcpu = rt_cpu_index(bind_cpu);

#if RT_THREAD_PRIORITY_MAX > 32
        pcpu->ready_table[thread->number] |= thread->high_mask;
#endif
        pcpu->priority_group |= thread->number_mask;

        rt_list_insert_before(&(rt_cpu_index(bind_cpu)->priority_table[thread->current_priority]),
                              &(thread->tlist));

        if (cpu_id != bind_cpu)
        {
            cpu_mask = 1 << bind_cpu;
            rt_hw_ipi_send(RT_SCHEDULE_IPI, cpu_mask);
        }
    }

    RT_DEBUG_LOG(RT_DEBUG_SCHEDULER, ("insert thread[%.*s], the priority: %d\n",
                                      RT_NAME_MAX, thread->name, thread->current_priority));

__exit:
    /* enable interrupt */
    rt_hw_interrupt_enable(level);
}
#else
void rt_schedule_insert_thread(struct rt_thread *thread)
{
    register rt_base_t temp;

    RT_ASSERT(thread != RT_NULL);

    /* disable interrupt */
    temp = rt_hw_interrupt_disable();

    /* it's current thread, it should be RUNNING thread */
    if (thread == rt_current_thread)
    {
        thread->stat = RT_THREAD_RUNNING | (thread->stat & ~RT_THREAD_STAT_MASK);
        goto __exit;
    }

    /* READY thread, insert to ready queue */
    thread->stat = RT_THREAD_READY | (thread->stat & ~RT_THREAD_STAT_MASK);//把线程状态的4位先清零，再置位
    /* insert thread to ready list */
    rt_list_insert_before(&(rt_thread_priority_table[thread->current_priority]),
                          &(thread->tlist));//插在链表的最后，节点的前面，双向链表

    RT_DEBUG_LOG(RT_DEBUG_SCHEDULER, ("insert thread[%.*s], the priority: %d\n",
                                      RT_NAME_MAX, thread->name, thread->current_priority));

    /* set priority mask */
#if RT_THREAD_PRIORITY_MAX > 32
    rt_thread_ready_table[thread->number] |= thread->high_mask;
#endif
    rt_thread_ready_priority_group |= thread->number_mask;

__exit:
    /* enable interrupt */
    rt_hw_interrupt_enable(temp);
}
#endif /*RT_USING_SMP*/

/*
 * This function will remove a thread from system ready queue.
 *
 * @param thread the thread to be removed
 *
 * @note Please do not invoke this function in user application.
 */
#ifdef RT_USING_SMP
void rt_schedule_remove_thread(struct rt_thread *thread)
{
    register rt_base_t level;

    RT_ASSERT(thread != RT_NULL);

    /* disable interrupt */
    level = rt_hw_interrupt_disable();

    RT_DEBUG_LOG(RT_DEBUG_SCHEDULER, ("remove thread[%.*s], the priority: %d\n",
                                      RT_NAME_MAX, thread->name,
                                      thread->current_priority));

    /* remove thread from ready list */
    rt_list_remove(&(thread->tlist));
    if (thread->bind_cpu == RT_CPUS_NR)
    {
        if (rt_list_isempty(&(rt_thread_priority_table[thread->current_priority])))
        {
#if RT_THREAD_PRIORITY_MAX > 32
            rt_thread_ready_table[thread->number] &= ~thread->high_mask;
            if (rt_thread_ready_table[thread->number] == 0)
            {
                rt_thread_ready_priority_group &= ~thread->number_mask;
            }
#else
            rt_thread_ready_priority_group &= ~thread->number_mask;
#endif
        }
    }
    else
    {
        struct rt_cpu *pcpu = rt_cpu_index(thread->bind_cpu);

        if (rt_list_isempty(&(pcpu->priority_table[thread->current_priority])))
        {
#if RT_THREAD_PRIORITY_MAX > 32
            pcpu->ready_table[thread->number] &= ~thread->high_mask;
            if (rt_thread_ready_table[thread->number] == 0)
            {
                pcpu->priority_group &= ~thread->number_mask;
            }
#else
            pcpu->priority_group &= ~thread->number_mask;
#endif
        }
    }

    /* enable interrupt */
    rt_hw_interrupt_enable(level);
}
#else
void rt_schedule_remove_thread(struct rt_thread *thread)
{
    register rt_base_t level;

    RT_ASSERT(thread != RT_NULL);

    /* disable interrupt */
    level = rt_hw_interrupt_disable();

    RT_DEBUG_LOG(RT_DEBUG_SCHEDULER, ("remove thread[%.*s], the priority: %d\n",
                                      RT_NAME_MAX, thread->name,
                                      thread->current_priority));

    /* remove thread from ready list */
    rt_list_remove(&(thread->tlist));
    if (rt_list_isempty(&(rt_thread_priority_table[thread->current_priority])))//链表为空
    {
#if RT_THREAD_PRIORITY_MAX > 32
    //当在该优先级序号链表上面没有就绪线程时，就将位图归零
        rt_thread_ready_table[thread->number] &= ~thread->high_mask;//优先级数组的线程序号掩码，8位
        if (rt_thread_ready_table[thread->number] == 0)
        {
            rt_thread_ready_priority_group &= ~thread->number_mask;//储存在线程中的该线程的优先级序号掩码，32位
        }
#else
        rt_thread_ready_priority_group &= ~thread->number_mask;
#endif
    }

    /* enable interrupt */
    rt_hw_interrupt_enable(level);
}
#endif /*RT_USING_SMP*/

/**
 * This function will lock the thread scheduler.
 */
#ifdef RT_USING_SMP
void rt_enter_critical(void)
{
    register rt_base_t level;
    struct rt_thread *current_thread;

    /* disable interrupt */
    level = rt_hw_local_irq_disable();

    current_thread = rt_cpu_self()->current_thread;
    /*
     * the maximal number of nest is RT_UINT16_MAX, which is big
     * enough and does not check here
     */

    /* lock scheduler for all cpus */
    if (current_thread->scheduler_lock_nest == !!current_thread->cpus_lock_nest)
    {
        rt_hw_spin_lock(&_rt_critical_lock);
    }

    /* lock scheduler for local cpu */
    current_thread->scheduler_lock_nest ++;

    /* enable interrupt */
    rt_hw_local_irq_enable(level);
}
#else
void rt_enter_critical(void)
{
    register rt_base_t level;

    /* disable interrupt */
    level = rt_hw_interrupt_disable();

    /*
     * the maximal number of nest is RT_UINT16_MAX, which is big
     * enough and does not check here
     */
    rt_scheduler_lock_nest ++;

    /* enable interrupt */
    rt_hw_interrupt_enable(level);
}
#endif /*RT_USING_SMP*/
RTM_EXPORT(rt_enter_critical);

/**
 * This function will unlock the thread scheduler.
 */
#ifdef RT_USING_SMP
void rt_exit_critical(void)
{
    register rt_base_t level;
    struct rt_thread *current_thread;

    /* disable interrupt */
    level = rt_hw_local_irq_disable();

    current_thread = rt_cpu_self()->current_thread;

    current_thread->scheduler_lock_nest --;

    if (current_thread->scheduler_lock_nest == !!current_thread->cpus_lock_nest)
    {
        rt_hw_spin_unlock(&_rt_critical_lock);
    }

    if (current_thread->scheduler_lock_nest <= 0)
    {
        current_thread->scheduler_lock_nest = 0;
        /* enable interrupt */
        rt_hw_local_irq_enable(level);

        rt_schedule();
    }
    else
    {
        /* enable interrupt */
        rt_hw_local_irq_enable(level);
    }
}
#else
void rt_exit_critical(void)
{
    register rt_base_t level;

    /* disable interrupt */
    level = rt_hw_interrupt_disable();

    rt_scheduler_lock_nest --;

    if (rt_scheduler_lock_nest <= 0)
    {
        rt_scheduler_lock_nest = 0;
        /* enable interrupt */
        rt_hw_interrupt_enable(level);

        rt_schedule();
    }
    else
    {
        /* enable interrupt */
        rt_hw_interrupt_enable(level);
    }
}
#endif /*RT_USING_SMP*/
RTM_EXPORT(rt_exit_critical);

/**
 * Get the scheduler lock level
 *
 * @return the level of the scheduler lock. 0 means unlocked.
 */
rt_uint16_t rt_critical_level(void)
{
#ifdef RT_USING_SMP
    struct rt_thread *current_thread = rt_cpu_self()->current_thread;

    return current_thread->scheduler_lock_nest;
#else
	return rt_scheduler_lock_nest;
#endif /*RT_USING_SMP*/
}
RTM_EXPORT(rt_critical_level);

/**@}*/
