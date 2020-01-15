
#include "vs1003.h"

typedef __packed struct//RIFF块
{
	unsigned int ChunkID;//chunk id;这里固定为"RIFF",即0x46464952
	unsigned int ChunkSize ;//集合大小;文件总大小-8
	unsigned int Format;//格式;WAVE,即0x45564157
}ChunkRIFF;

typedef __packed struct//FMT块
{
	unsigned int ChunkID;//chunk id;这里固定为"fmt ",即0x20746D66
	unsigned int ChunkSize ;//子集合大小(不包括ID和Size)
	unsigned short AudioFormat;//音频格式;0X01,表示线性PCM;0X11表示IMA ADPCM
	unsigned short NumOfChannels;//通道数量 1表示单声道 2表示双声道
	unsigned int SampleRate;//采样率
	unsigned int ByteRate;//字节速率
	unsigned short BlockAlign;//块对齐(字节)
	unsigned short BitsPerSample;//单个采样数据大小;4位ADPCM,设置为4
}ChunkFMT;

typedef __packed struct//DATA块
{
	unsigned int ChunkID;//chunk id;这里固定为"data",即0x61746164
	unsigned int ChunkSize;//子集合大小(不包括ID和Size)
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
	int  fd;//文件句柄
	__WaveHeader wav_head;
	char record_data[RECORDER_LEN];
}vs_recorder;


