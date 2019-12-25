#include "audio_common.h"
#include <rtthread.h>

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
    char file_path[256];
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
        rt_memset(file_path,0,256);
        if (rt_mq_recv(music_mq, (void*)&mb_message, sizeof(MESSAGE),1000) != RT_EOK)
        {
            int ret;
            ret = auto_play("/music/",file_path);
            if(ret)
                continue;
        }
        rt_sem_control(music_break,RT_IPC_CMD_RESET,0);//½«ÐÅºÅÁ¿¹éÁã
        AudioPlayFile(rt_strlen(file_path) >0 ? file_path : mb_message.file_name);
    }
}

void music_startup(void)
{
    rt_thread_t tid;
    
    tid = rt_thread_find("music");
    if(tid)
    {
        rt_kprintf("runing this task\n");
        return;
    }
        
    tid = rt_thread_create("music",
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
    rt_thread_t tid;
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
    tid = rt_thread_find("music");
    if(!tid)
    {
        rt_kprintf("please run this task\n");
        return -1;
    }
    if(!strncmp(argv[1],"song",rt_strlen("song")))
    {
        rt_err_t err;
        if(argc < 3)
        {
            music_log("please input the name of songs\n");
            return -RT_ERROR;
        }
        rt_sem_release(dma_int);
        rt_sem_release(music_break);
        rt_memset(&message_oo,0,sizeof(MESSAGE));
        rt_strncpy(message_oo.file_name,argv[2],rt_strlen(argv[2]));
        err = rt_mq_send(music_mq,(void*)&message_oo,sizeof(MESSAGE));
        if(err == RT_EOK)
        {
            //music_log("rt_mq_send done!\n");
        }
        else
        {
            //music_log("rt_mq_send fail!\n");
            return -RT_ERROR;
        }
        return RT_EOK;
    }
    if(!strncmp(argv[1],"stop",rt_strlen("stop")))
    {
        rt_err_t err;
        rt_sem_release(dma_int);
        rt_sem_release(music_break);
        return RT_EOK;
    }
 return 0;
}
void dac_cmd(int argc, char ** argv)
{
    if (argc != 2)
    {
        music_log("Usage: err\n");
        return;// -RT_ERROR;
    }
    if(!strncmp(argv[1],"run",rt_strlen("run")))
    {
        Play_Start();
        //return RT_EOK;
    }
    if(!strncmp(argv[1],"stop",rt_strlen("st")))
    {
        Play_Stop();
        //return RT_EOK;
    }
}
void dir_cmd(int argc, char ** argv)
{
    if (argc != 2)
    {
        music_log("Usage: err\n");
        return;// -RT_ERROR;
    }
    //auto_play(argv[1]);
}

//INIT_APP_EXPORT(music_startup);

#ifdef RT_USING_FINSH
#include <finsh.h>
MSH_CMD_EXPORT_ALIAS(music_startup, music, ----music_startup----);
MSH_CMD_EXPORT_ALIAS(music_action,  p,  ----music play control----);
MSH_CMD_EXPORT_ALIAS(dac_cmd, dac_cmd, ----dac_cmd----);
//MSH_CMD_EXPORT_ALIAS(dir_cmd, dir, ----dac_cmd----);

#endif

