#include "audio_common.h"
char hellos[630*1024];//__attribute__((section("EXRAM")));

#define MUSIC_DEBUG
#ifdef MUSIC_DEBUG
#define music_log(fmt,arg...) rt_kprintf("MUSIC->%s - L:%d  "fmt"\n",__FUNCTION__, __LINE__, ##arg);
#else
#define music_log
#endif

typedef struct 
{
    char file_name[50];
}MESSAGE;

#define RT_MUSIC_THREAD_PRIORITY 5

static rt_mq_t music_mq = RT_NULL;
rt_sem_t dma_int = RT_NULL;
rt_sem_t music_break = RT_NULL;

static MESSAGE mb_message;

static void music_service_task(void *param)
{
    
    AudioPlay_Init();
    rt_memset(&mb_message,0,sizeof(MESSAGE));
    music_mq = rt_mq_create("music_mq",100,1,RT_IPC_FLAG_FIFO);
    if (music_mq == RT_NULL)
    {
        music_log("music_mb create fail!\n");
        return ;
    }
    dma_int = rt_sem_create("dma_int",0,RT_IPC_FLAG_FIFO);
    if (dma_int == RT_NULL)
    {
        music_log("dma_int create fail!\n");
        return ;
    }
    music_break = rt_sem_create("music_break",0,RT_IPC_FLAG_FIFO);
    if (music_break == RT_NULL)
    {
        music_log("music_break create fail!\n");
        return ;
    }
    
    while (1)
    {
        if (rt_mq_recv(music_mq, (void*)&mb_message, sizeof(MESSAGE),RT_WAITING_FOREVER) != RT_EOK)
        {
            music_log("music_mQ rt_mb_recv fail!\n");
            continue;
        }
        music_log("runing song %s\n",mb_message.file_name);
        rt_sem_control(music_break,RT_IPC_CMD_RESET,0);//Ω´–≈∫≈¡øπÈ¡„
        AudioPlayFile(mb_message.file_name);
    }
}

void music_startup(void)
{
    rt_thread_t tid;

    tid = rt_thread_create("music_service_task",
                           music_service_task, 
                           (void *) 0,
                           2048*5,
                           RT_MUSIC_THREAD_PRIORITY,
                           20);
    if (tid != RT_NULL)
    {
        rt_thread_startup(tid);
        music_log("music_service_task init done...\n");
    }
    else
    {
        music_log("music_service_task init fail...\n");
    }
}
int music_action(int argc, char ** argv)
{
    MESSAGE message_oo;
    if(!music_mq)
    {
        music_log("please init music service task first\n");
        return -RT_ERROR;
    }
    if (argc < 2)
    {
        music_log("Usage: err\n");
        return -RT_ERROR;
    }
    if(!strncmp(argv[1],"song",rt_strlen("song")))
    {
        rt_err_t err;
        if(argc < 3)
        {
            music_log("please input the name of songs\n");
            return -RT_ERROR;
        }
        rt_memset(&message_oo,0,sizeof(MESSAGE));
        rt_strncpy(message_oo.file_name,argv[2],rt_strlen(argv[2]));
        err = rt_mq_send(music_mq,(void*)&message_oo,sizeof(MESSAGE));
        if(err == RT_EOK)
        {
            music_log("rt_mq_send done!\n");
        }
        else
        {
            music_log("rt_mq_send fail!\n");
            return -RT_ERROR;
        }
        return RT_EOK;
    }
    if(!strncmp(argv[1],"stop",rt_strlen("stop")))
    {
        rt_err_t err;
        rt_sem_release(dma_int);
        rt_sem_release(music_break);
        Play_Stop();
        return RT_EOK;
    }
 return 0;
}
#ifdef RT_USING_FINSH
#include <finsh.h>
MSH_CMD_EXPORT_ALIAS(music_startup, music, ----music_startup----);
MSH_CMD_EXPORT_ALIAS(music_action,  play,  ----music play control----);

#endif

