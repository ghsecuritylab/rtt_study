#ifndef VS10XX_H
#define VS10XX_H

/* Include processor definition */
#include "sys.h"
#include "spi.h"
#include <stdlib.h>
#include <rtthread.h>
//vs1003寄存器相关宏定义 
#define VS_WRITE_COMMAND 0x02    //写命令
#define VS_READ_COMMAND  0x03    //读命令
#define SPI_MODE         0x00    //控制模式   
#define SPI_STATUS       0x01    //VS1003 状态   
#define SPI_BASS         0x02    //内置高/低音增强器   
#define SPI_CLOCKF       0x03    //时钟频率 + 倍频数   
#define SPI_DECODE_TIME  0x04    //每秒解码次数  
#define SPI_AUDATA       0x05    //音频数据   
#define SPI_WRAM         0x06    //RAM 读写  
#define SPI_WRAMADDR     0x07    //RAM 读写地址  
#define SPI_HDAT0        0x08    //流头数据0  
#define SPI_HDAT1        0x09    //流头数据1  
#define SPI_AIADDR       0x0A    //应用程序起始地址  
#define SPI_VOL          0x0B    //音量控制  
#define SPI_AICTRL0      0x0C    //应用控制寄存器0 
#define SPI_AICTRL1      0x0D    //应用控制寄存器1   
#define SPI_AICTRL2      0x0E    //应用控制寄存器2  
#define SPI_AICTRL3      0x0F    //应用控制寄存器3  
/*#define SM_DIFF         0x01   
#define SM_JUMP         0x02   
#define SM_RESET        0x04   
#define SM_OUTOFWAV     0x08   
#define SM_PDOWN        0x10   
#define SM_TESTS        0x20   
#define SM_STREAM       0x40   
#define SM_PLUSV        0x80   
#define SM_DACT         0x100   
#define SM_SDIORD       0x200   
#define SM_SDISHARE     0x400   
#define SM_SDINEW       0x800   
#define SM_ADPCM        0x1000   
#define SM_ADPCM_HP     0x2000 */

void VS_Reset(); //VS1003软复位及初始化
void VS_Write_Reg(unsigned char addr,unsigned char hdat,unsigned char ldat); //向VS1003的功能寄存器写入一个字
unsigned int VS_Read_Reg(unsigned char addr); //从VS1003的功能寄存器读取一个字
void VS_Send_Dat(unsigned char dat); //向VS1003发送音频数据
void VS_Flush_Buffer(); //清空VS1003的数据缓冲区
void VS_sin_test(unsigned char x); //正弦测试
void LoadPatch(); //为VS1003打补丁

void vs_spi_dma_stop();
void vs_xdcs_cs(u8 status);
int dev_audio_play(char* file_name);
int vs_auto_play(char * file_dir_name);
#endif