#ifndef __DRV_LCD_H__
#define __DRV_LCD_H__		

#include <rtdevice.h>
#include "stm32f4xx.h"
										  
int rt_lcd_init(void);
#define	LCD_BACK PFout(10)  //LCD±³¹â    	PF10 	  

#endif  
	 
	 



