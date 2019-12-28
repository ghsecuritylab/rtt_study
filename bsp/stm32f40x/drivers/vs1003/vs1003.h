#ifndef VS10XX_H
#define VS10XX_H

/* Include processor definition */
#include "sys.h"
#include "spi.h"
#include <stdlib.h>
#include <rtthread.h>
//vs1003�Ĵ�����غ궨�� 
#define VS_WRITE_COMMAND 0x02    //д����
#define VS_READ_COMMAND  0x03    //������
#define SPI_MODE         0x00    //����ģʽ   
#define SPI_STATUS       0x01    //VS1003 ״̬   
#define SPI_BASS         0x02    //���ø�/������ǿ��   
#define SPI_CLOCKF       0x03    //ʱ��Ƶ�� + ��Ƶ��   
#define SPI_DECODE_TIME  0x04    //ÿ��������  
#define SPI_AUDATA       0x05    //��Ƶ����   
#define SPI_WRAM         0x06    //RAM ��д  
#define SPI_WRAMADDR     0x07    //RAM ��д��ַ  
#define SPI_HDAT0        0x08    //��ͷ����0  
#define SPI_HDAT1        0x09    //��ͷ����1  
#define SPI_AIADDR       0x0A    //Ӧ�ó�����ʼ��ַ  
#define SPI_VOL          0x0B    //��������  
#define SPI_AICTRL0      0x0C    //Ӧ�ÿ��ƼĴ���0 
#define SPI_AICTRL1      0x0D    //Ӧ�ÿ��ƼĴ���1   
#define SPI_AICTRL2      0x0E    //Ӧ�ÿ��ƼĴ���2  
#define SPI_AICTRL3      0x0F    //Ӧ�ÿ��ƼĴ���3  
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

void VS_Reset(); //VS1003��λ����ʼ��
void VS_Write_Reg(unsigned char addr,unsigned char hdat,unsigned char ldat); //��VS1003�Ĺ��ܼĴ���д��һ����
unsigned int VS_Read_Reg(unsigned char addr); //��VS1003�Ĺ��ܼĴ�����ȡһ����
void VS_Send_Dat(unsigned char dat); //��VS1003������Ƶ����
void VS_Flush_Buffer(); //���VS1003�����ݻ�����
void VS_sin_test(unsigned char x); //���Ҳ���
void LoadPatch(); //ΪVS1003�򲹶�

void vs_spi_dma_stop();
void vs_xdcs_cs(u8 status);
int dev_audio_play(char* file_name);
int vs_auto_play(char * file_dir_name);
#endif