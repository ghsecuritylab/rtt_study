
#include "vs1003.h"

typedef __packed struct//RIFF��
{
	unsigned int ChunkID;//chunk id;����̶�Ϊ"RIFF",��0x46464952
	unsigned int ChunkSize ;//���ϴ�С;�ļ��ܴ�С-8
	unsigned int Format;//��ʽ;WAVE,��0x45564157
}ChunkRIFF;

typedef __packed struct//FMT��
{
	unsigned int ChunkID;//chunk id;����̶�Ϊ"fmt ",��0x20746D66
	unsigned int ChunkSize ;//�Ӽ��ϴ�С(������ID��Size)
	unsigned short AudioFormat;//��Ƶ��ʽ;0X01,��ʾ����PCM;0X11��ʾIMA ADPCM
	unsigned short NumOfChannels;//ͨ������ 1��ʾ������ 2��ʾ˫����
	unsigned int SampleRate;//������
	unsigned int ByteRate;//�ֽ�����
	unsigned short BlockAlign;//�����(�ֽ�)
	unsigned short BitsPerSample;//�����������ݴ�С;4λADPCM,����Ϊ4
}ChunkFMT;

typedef __packed struct//DATA��
{
	unsigned int ChunkID;//chunk id;����̶�Ϊ"data",��0x61746164
	unsigned int ChunkSize;//�Ӽ��ϴ�С(������ID��Size)
}ChunkHDR;

typedef __packed struct
{ 
	ChunkRIFF riff;
	ChunkFMT fmt;
	ChunkHDR data;
}__WaveHeader;
#define RECORDER_LEN 512
typedef __packed struct
{ 
	char is_playing;
	int  len;
	int  fd;//�ļ����
	__WaveHeader wav_head;
	char record_data[RECORDER_LEN];
}vs_recorder;


