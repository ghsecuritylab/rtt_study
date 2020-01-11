#ifndef __SRAM_VARIABLE_H__
#define __SRAM_VARIABLE_H__		
#include "drv_lcd.h"

#define LCD_HIGH_1 480//ÆÁ¸ß¶È
#define LCD_WIDTH_1 480//ÆÁ¿í¶È

//extern u8 sram_test_data[100*1024];
extern u16 sramlcdbuf[LCD_HIGH_1*LCD_WIDTH_1];//SRAM LCD BUFFER,±³¾°Í¼Æ¬ÏÔ´æÇø 


#endif