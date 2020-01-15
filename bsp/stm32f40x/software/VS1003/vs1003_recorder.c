#include "vs1003_recorder.h"
#include <dfs.h>
#include <dfs_posix.h>

vs_recorder vs_record;

void VS_WR_Cmd(unsigned char addr, int value)
{
    unsigned char hbat;
    unsigned char lbat;
    hbat = value>>8;
    lbat = value;
    VS_Write_Reg(addr,hbat,lbat);
    return ;
}
//激活PCM 录音模式
//agc:0,自动增益.1024相当于1倍,512相当于0.5倍,最大值65535=64倍		  
void recoder_enter_rec_mode(u16 agc)
{
	//如果是IMA ADPCM,采样率计算公式如下:
 	//采样率=CLKI/256*d;	
	//假设d=0,并2倍频,外部晶振为12.288M.那么Fc=(2*12288000)/256*6=16Khz
	//如果是线性PCM,采样率直接就写采样值   
 	VS_WR_Cmd(SPI_AICTRL0,8000);	//设置采样率,设置为8Khz
 	VS_WR_Cmd(SPI_AICTRL1,agc);		//设置增益,0,自动增益.1024相当于1倍,512相当于0.5倍,最大值65535=64倍	
 	VS_WR_Cmd(SPI_AICTRL2,0);		//设置增益最大值,0,代表最大值65536=64X
 	VS_WR_Cmd(SPI_AICTRL3,6);		//左通道(MIC单声道输入)
	VS_WR_Cmd(SPI_CLOCKF,0X2000);	//置VS10XX的时钟,MULT:2倍频;ADD:不允许;CLK:12.288Mhz
	VS_WR_Cmd(SPI_MODE,0x3804);		//MIC,录音激活    
 	rt_thread_delay(2);					//等待至少1.35ms 
}
 
//初始化WAV头.
void recoder_wav_init(__WaveHeader* wavhead) //初始化WAV头			   
{
	wavhead->riff.ChunkID=0X46464952;	//"RIFF"
	wavhead->riff.ChunkSize=0;			//还未确定,最后需要计算
	wavhead->riff.Format=0X45564157; 	//"WAVE"
	wavhead->fmt.ChunkID=0X20746D66; 	//"fmt "
	wavhead->fmt.ChunkSize=16; 			//大小为16个字节
	wavhead->fmt.AudioFormat=0X01; 		//0X01,表示PCM;0X01,表示IMA ADPCM
 	wavhead->fmt.NumOfChannels=1;		//单声道
 	wavhead->fmt.SampleRate=8000;		//8Khz采样率 采样速率
 	wavhead->fmt.ByteRate=wavhead->fmt.SampleRate*2;//16位,即2个字节
 	wavhead->fmt.BlockAlign=2;			//块大小,2个字节为一个块
 	wavhead->fmt.BitsPerSample=16;		//16位PCM
   	wavhead->data.ChunkID=0X61746164;	//"data"
 	wavhead->data.ChunkSize=0;			//数据大小,还需要计算  
}
int dev_record_init(char *file_name)
{
    int fd;
    int ret;
    rt_memset(&vs_record,0,sizeof(vs_recorder));
    recoder_wav_init(&vs_record.wav_head);
    fd = open(file_name,O_RDWR | O_CREAT);
    if(fd<0)
    {
        rt_kprintf("dev_record_init creat file %s fail\n",file_name);
        return -1;
    }
    vs_record.fd = fd;
    ret = write(vs_record.fd,&vs_record.wav_head,sizeof(__WaveHeader));
    if(ret!=sizeof(__WaveHeader))
    {
        rt_kprintf("write  filefail\n");
        return -1;
    }
    if(vs_get_status()==1)
    {
        vs_record.is_playing = 1;
    }
    vs_set_status(2);//关闭播放，进入录音模式
    recoder_enter_rec_mode(1024*4);
    rt_kprintf("init ok\n");
}
int dev_record_doing(void)
{
    int len,temp = 0,i=0;
    int ret;
    len = VS_Read_Reg(SPI_HDAT1);
    if(len > 256)
    {
        rt_kprintf("len too long :%d\n",len);
        len = 256;
    }
    while(len)
    {
        temp = VS_Read_Reg(SPI_HDAT0);
        vs_record.record_data[i++] = temp & 0xff;
        vs_record.record_data[i++] = (temp>>8) & 0xff;
        len--;
    }
    if(vs_record.fd)
    {
        ret = write(vs_record.fd,vs_record.record_data,len*2);
        if(ret!=len*2)
        {
            rt_kprintf("write err:%d\n",ret);
        }
    }
    vs_record.len += len*2;
    //rt_kprintf("len:%d len:%d\n",len,vs_record.len);
    return 0;
}
int dev_record_close(void)
{
    VS_WR_Cmd(SPI_MODE,0x0804);		//MIC,录音激活 
    vs_record.wav_head.riff.ChunkSize = vs_record.len + 36;
    vs_record.wav_head.data.ChunkSize = vs_record.len;
    lseek(vs_record.fd,0,SEEK_SET);
    write(vs_record.fd,&vs_record.wav_head,sizeof(__WaveHeader));
    close(vs_record.fd);
    if(vs_record.is_playing)
    {
        vs_set_status(1);
    }
    else
    {
        vs_set_status(0);
    }
    return 0;
}
