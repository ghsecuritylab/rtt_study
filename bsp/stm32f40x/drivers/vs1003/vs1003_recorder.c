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
//����PCM ¼��ģʽ
//agc:0,�Զ�����.1024�൱��1��,512�൱��0.5��,���ֵ65535=64��		  
void recoder_enter_rec_mode(u16 agc)
{
	//�����IMA ADPCM,�����ʼ��㹫ʽ����:
 	//������=CLKI/256*d;	
	//����d=0,��2��Ƶ,�ⲿ����Ϊ12.288M.��ôFc=(2*12288000)/256*6=16Khz
	//���������PCM,������ֱ�Ӿ�д����ֵ   
 	VS_WR_Cmd(SPI_AICTRL0,8000);	//���ò�����,����Ϊ8Khz
 	VS_WR_Cmd(SPI_AICTRL1,agc);		//��������,0,�Զ�����.1024�൱��1��,512�൱��0.5��,���ֵ65535=64��	
 	VS_WR_Cmd(SPI_AICTRL2,0);		//�����������ֵ,0,�������ֵ65536=64X
 	VS_WR_Cmd(SPI_AICTRL3,6);		//��ͨ��(MIC����������)
	VS_WR_Cmd(SPI_CLOCKF,0X2000);	//��VS10XX��ʱ��,MULT:2��Ƶ;ADD:������;CLK:12.288Mhz
	VS_WR_Cmd(SPI_MODE,0x3804);		//MIC,¼������    
 	rt_thread_delay(2);					//�ȴ�����1.35ms 
}
 
//��ʼ��WAVͷ.
void recoder_wav_init(__WaveHeader* wavhead) //��ʼ��WAVͷ			   
{
	wavhead->riff.ChunkID=0X46464952;	//"RIFF"
	wavhead->riff.ChunkSize=0;			//��δȷ��,�����Ҫ����
	wavhead->riff.Format=0X45564157; 	//"WAVE"
	wavhead->fmt.ChunkID=0X20746D66; 	//"fmt "
	wavhead->fmt.ChunkSize=16; 			//��СΪ16���ֽ�
	wavhead->fmt.AudioFormat=0X01; 		//0X01,��ʾPCM;0X01,��ʾIMA ADPCM
 	wavhead->fmt.NumOfChannels=1;		//������
 	wavhead->fmt.SampleRate=8000;		//8Khz������ ��������
 	wavhead->fmt.ByteRate=wavhead->fmt.SampleRate*2;//16λ,��2���ֽ�
 	wavhead->fmt.BlockAlign=2;			//���С,2���ֽ�Ϊһ����
 	wavhead->fmt.BitsPerSample=16;		//16λPCM
   	wavhead->data.ChunkID=0X61746164;	//"data"
 	wavhead->data.ChunkSize=0;			//���ݴ�С,����Ҫ����  
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
    vs_set_status(2);//�رղ��ţ�����¼��ģʽ
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
    VS_WR_Cmd(SPI_MODE,0x0804);		//MIC,¼������ 
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
