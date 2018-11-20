#ifndef __ADC_H
#define __ADC_H	
#include "sys.h"


extern volatile uint16_t AD_value[4];

/*************************** 文件说明 *******************************/
//此程序适用于七彩光子太极M3—STM32F103开发板及部分兼容的STM32系统，
//淘宝搜索"七彩光子 太极M3 STM32开发板"购买超高性价比开发板或获取更多资料
//本程序部分资源来自网络,只供学习使用，未经作者许可，不得用于其它任何用途！
/********************************************************************/ 



void ADC1_Init(void);
u16  Get_Adc(u8 ch); 
u16 Get_Adc_Average(u8 ch,u8 times); 
void ADC_DMA_Init(void);
#endif 
