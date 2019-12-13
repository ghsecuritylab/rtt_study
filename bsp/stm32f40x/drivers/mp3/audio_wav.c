#include "music.h"
#include "audio_wav.h"
#include <string.h>

AudioPlayRes WAV_GetInfo(int fd,Wav_Info* info)
{
	uint8_t buf[64];
	int br;
	uint32_t fptr = 0;
	
	ChunkFMT *fmt;
	ChunkHDR *header;
	
	lseek(fd,0,SEEK_SET);
	br = read(fd,buf,sizeof(buf));//读取32字节
	
	fptr = 12;//最开始是RIFF块 12bytes
	
	if(((ChunkRIFF*)buf)->ChunkID == 0x46464952 && ((ChunkRIFF*)buf)->Format == 0x45564157)//如果块ID为RIFF 格式是WAVE
	{
		fmt = (ChunkFMT*)(buf + 12);//读取FMT块
		if(fmt->ChunkID == 0x20746D66)//如果是FMT块
		{
			info->AudioFormat=fmt->AudioFormat;//音频格式
			info->nChannels=fmt->NumOfChannels;//通道数
			info->Samplerate=fmt->SampleRate;//采样率
			info->Bitrate=fmt->ByteRate*8;//得到位速
			info->BlockAlign=fmt->BlockAlign;//块对齐
			info->Bps=fmt->BitsPerSample;//位数 16/24/32位
			uart_printf("音频格式==%04x\n",info->AudioFormat);
			uart_printf("通道数==%d\n",info->nChannels);
			uart_printf("采样率==%d\n",info->Samplerate);
			uart_printf("得到位速==%d\n",info->Bitrate);
			uart_printf("块对齐==%d\n",info->BlockAlign);
			uart_printf("位数==%d\n",info->Bps);
		}
		else
		{
			return AudioPlay_UnsupportedFormat;//文件格式不是WAVE
		}
		
		fptr += fmt->ChunkSize + 8;//还有文件头
		
		while(1)
		{
			lseek(fd,fptr,SEEK_SET);
			br = read(fd,buf,sizeof(buf));//读取32字节
			header = (ChunkHDR*) buf;
			
			if(header->ChunkID == 0x61746164)
			{
				info->DataStartOffset = fptr + 8;//跳过头
				break;//找到数据了
			}
			
			fptr += header->ChunkSize + 8;//继续寻找下一个 别忘了文件头的大小
			
			if(fptr > 4096)//找了4k还没有找到
				return AudioPlay_UnsupportedFormat;//文件格式不是WAVE
		}
	}
	else
	{
		return AudioPlay_UnsupportedFormat;//不是RIFF文件
	}
	uart_printf("数据偏移==%d\n",info->DataStartOffset);
	return AudioPlay_OK;
}

AudioPlayRes WAV_Play(char* path)
{
	AudioPlayRes res = AudioPlay_OK;
	Wav_Info __info;
	Wav_Info* WavInfo = NULL;
	int br = 0xFFFF;
	int fd;
	WavInfo = &__info;
	fd = open(path,O_RDONLY);
	if(fd == -1)//打开文件
	{
		res =  AudioPlay_OpenFileError;//打开文件错误
		uart_printf("打开文件错误\n");
	}
	else//打开成功
	{
		res = WAV_GetInfo(fd,WavInfo);//获取文件信息
		
		if(res == AudioPlay_OK)
		{			
			AudioPlayInfo.FileType = AudioFileType_WAV;
			AudioPlayInfo.Channels = WavInfo->nChannels;
			
			AudioPlayInfo.Samplerate = WavInfo->Samplerate;
			AudioPlayInfo.Bitrate = WavInfo->Bitrate/1000;
			AudioPlayInfo.TotalSec = (lseek(fd,0,SEEK_END) - WavInfo->DataStartOffset) / (WavInfo->Bitrate / WavInfo->Bps * WavInfo->nChannels);
			AudioPlayInfo.BufferSize = DAC_Buffer_Size * 2;

			AudioPlayInfo.Flags |= AUDIO_FLAG_INFO_READY;
			//AudioPlay_Init();//xqy添加，ADC选择定时器为ADC转换触发源
			if(AudioPlay_Config(WavInfo->Bps,WavInfo->Samplerate,DAC_Buffer_Size))//配置DMA等数据
			{
				res = AudioPlay_UnsupportedParameter;//数据格式不支持
			}
		}
	}
	
	if(res == AudioPlay_OK)
	{
		lseek(fd,WavInfo->DataStartOffset,SEEK_SET);//定位到PWM数据的开始地方
		Audio_first_play(fd);
	}
	
	while(res == AudioPlay_OK)
	{
		if(br < DAC_Buffer_Size * 2)//读完文件了
		{
			res = AudioPlay_PlayEnd;
			uart_printf("播放结束\n");
			break;
		}
		
		AudioPlayInfo.CurrentSec = (lseek(fd,0,SEEK_CUR) - WavInfo->DataStartOffset) / (WavInfo->Bitrate / WavInfo->Bps * WavInfo->nChannels);//计算播放时间
		
		//AudioPlay_PendSem();
		rt_sem_take(dma_int,RT_WAITING_FOREVER);
		rt_sem_control(dma_int,RT_IPC_CMD_RESET,0);//将信号量归零
		{
		    rt_err_t err;
		    err = rt_sem_trytake(music_break);
		    if(err==RT_EOK)
		    {
		        rt_kprintf("exiting music player\n");
		        break;
		    }
		}
		br = read(fd,(unsigned char*)AudioPlay_GetCurrentBuff(),DAC_Buffer_Size*4/2);//填充缓冲区
		AudioPlay_DataProc(AudioPlay_GetCurrentBuff(),DAC_Buffer_Size);
	}
	
	Play_Stop();
	close(fd);//关闭文件
	return res;
}
