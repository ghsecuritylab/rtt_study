#include "music.h"
#include "audio_wav.h"
#include <string.h>

AudioPlayRes WAV_GetInfo(FIL* file,Wav_Info* info)
{
	uint8_t buf[64];
	uint32_t br;
	uint32_t fptr = 0;
	
	ChunkFMT *fmt;
	ChunkHDR *header;
	
	f_lseek(file,0);
	f_read(file,buf,sizeof(buf),&br);//��ȡ32�ֽ�
	
	fptr = 12;//�ʼ��RIFF�� 12bytes
	
	if(((ChunkRIFF*)buf)->ChunkID == 0x46464952 && ((ChunkRIFF*)buf)->Format == 0x45564157)//�����IDΪRIFF ��ʽ��WAVE
	{
		fmt = (ChunkFMT*)(buf + 12);//��ȡFMT��
		if(fmt->ChunkID == 0x20746D66)//�����FMT��
		{
			info->AudioFormat=fmt->AudioFormat;//��Ƶ��ʽ
			info->nChannels=fmt->NumOfChannels;//ͨ����
			info->Samplerate=fmt->SampleRate;//������
			info->Bitrate=fmt->ByteRate*8;//�õ�λ��
			info->BlockAlign=fmt->BlockAlign;//�����
			info->Bps=fmt->BitsPerSample;//λ�� 16/24/32λ
			uart_printf("��Ƶ��ʽ==%04x\n",info->AudioFormat);
			uart_printf("ͨ����==%d\n",info->nChannels);
			uart_printf("������==%d\n",info->Samplerate);
			uart_printf("�õ�λ��==%d\n",info->Bitrate);
			uart_printf("�����==%d\n",info->BlockAlign);
			uart_printf("λ��==%d\n",info->Bps);
		}
		else
		{
			return AudioPlay_UnsupportedFormat;//�ļ���ʽ����WAVE
		}
		
		fptr += fmt->ChunkSize + 8;//�����ļ�ͷ
		
		while(1)
		{
			f_lseek(file,fptr);
			f_read(file,buf,sizeof(buf),&br);//��ȡ32�ֽ�
			header = (ChunkHDR*) buf;
			
			if(header->ChunkID == 0x61746164)
			{
				info->DataStartOffset = fptr + 8;//����ͷ
				break;//�ҵ�������
			}
			
			fptr += header->ChunkSize + 8;//����Ѱ����һ�� �������ļ�ͷ�Ĵ�С
			
			if(fptr > 4096)//����4k��û���ҵ�
				return AudioPlay_UnsupportedFormat;//�ļ���ʽ����WAVE
		}
	}
	else
	{
		return AudioPlay_UnsupportedFormat;//����RIFF�ļ�
	}
	uart_printf("����ƫ��==%d\n",info->DataStartOffset);
	return AudioPlay_OK;
}

AudioPlayRes WAV_Play(char* path)
{
	AudioPlayRes res = AudioPlay_OK;
	Wav_Info __info;
	Wav_Info* WavInfo = NULL;
	FIL* Wav_File = NULL;
	UINT br = 0xFFFF;
	
	Wav_File = &AudioFile;
	WavInfo = &__info;
	
	if(f_open(Wav_File,path,FA_READ))//���ļ�
	{
		res =  AudioPlay_OpenFileError;//���ļ�����
		uart_printf("���ļ�����\n");
	}
	else//�򿪳ɹ�
	{
		res = WAV_GetInfo(Wav_File,WavInfo);//��ȡ�ļ���Ϣ
		
		if(res == AudioPlay_OK)
		{			
			AudioPlayInfo.FileType = AudioFileType_WAV;
			AudioPlayInfo.Channels = WavInfo->nChannels;
			
			AudioPlayInfo.Samplerate = WavInfo->Samplerate;
			AudioPlayInfo.Bitrate = WavInfo->Bitrate/1000;
			AudioPlayInfo.TotalSec = (f_size(Wav_File) - WavInfo->DataStartOffset) / (WavInfo->Bitrate / WavInfo->Bps * WavInfo->nChannels);
			AudioPlayInfo.BufferSize = DAC_Buffer_Size * 2;

			AudioPlayInfo.Flags |= AUDIO_FLAG_INFO_READY;
			//AudioPlay_Init();//xqy���ӣ�ADCѡ��ʱ��ΪADCת������Դ
			if(AudioPlay_Config(WavInfo->Bps,WavInfo->Samplerate,DAC_Buffer_Size))//����DMA������
			{
				res = AudioPlay_UnsupportedParameter;//���ݸ�ʽ��֧��
			}
		}
	}
	
	if(res == AudioPlay_OK)
	{
		f_lseek(Wav_File,WavInfo->DataStartOffset);//��λ��PWM���ݵĿ�ʼ�ط�
		//Play_Start();
		Audio_first_play(Wav_File);
	}
	
	while(res == AudioPlay_OK)
	{
		if(br < DAC_Buffer_Size * 2)//�����ļ���
		{
			res = AudioPlay_PlayEnd;
			uart_printf("���Ž���\n");
			break;
		}
		
		AudioPlayInfo.CurrentSec = (Wav_File->fptr - WavInfo->DataStartOffset) / (WavInfo->Bitrate / WavInfo->Bps * WavInfo->nChannels);//���㲥��ʱ��
		
		//AudioPlay_PendSem();
		rt_sem_take(dma_int,RT_WAITING_FOREVER);
		rt_sem_control(dma_int,RT_IPC_CMD_RESET,0);//���ź�������
		{
		    rt_err_t err;
		    err = rt_sem_trytake(music_break);
		    if(err==RT_EOK)
		    {
		        rt_kprintf("exiting music player\n");
		        break;
		    }
		}
		f_read(Wav_File,(unsigned char*)AudioPlay_GetCurrentBuff(),DAC_Buffer_Size*4/2,&br);//��仺����
		AudioPlay_DataProc(AudioPlay_GetCurrentBuff(),DAC_Buffer_Size);
	}
	
	Play_Stop();
	f_close(Wav_File);//�ر��ļ�
	return res;
}