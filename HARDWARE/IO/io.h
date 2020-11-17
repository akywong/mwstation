#ifndef __IO_H_
#define __IO_H_

#include "sys.h"

extern unsigned char ReadConversionData;
/***************** IO操作函数宏定义 *********************/
#define IO_ON(x)      x=1  
#define IO_OFF(x)     x=0
#define IO_TOGGLE(x)  x^=1
/****************************** end *********************************/

/*#define CONFIG_GPIO     GPIOA
#define CONFIG_PIN			GPIO_Pin_1
#define CONFIG_IO_RCC_CLK RCC_APB2Periph_GPIOA
#define CONFIG_IO_GET_IN()  (0)//((CONFIG_GPIO->IDR & CONFIG_PIN)?(1):(0))*/

//ADS1220 RDY 中断
#define ADS1220_DRY_RCC_CLK  RCC_APB2Periph_GPIOG
#define ADS1220_DRY_GPIO GPIOG
#define ADS1220_DRY_PIN GPIO_Pin_4
#define ADS1220_DRY_GET_IN()  ((ADS1220_DRY_GPIO->IDR & ADS1220_DRY_PIN)?(1):(0))

//LPS22HB DRY
#define LPS22HB_DRY_RCC_CLK  RCC_APB2Periph_GPIOG
#define LPS22HB_DRY_GPIO GPIOG
#define LPS22HB_DRY_PIN GPIO_Pin_7
#define LPS22HB_DRY_GET_IN()  ((LPS22HB_DRY_GPIO->IDR & LPS22HB_DRY_PIN)?(1):(0))

/************************* 函数声明 *********************************/
void IO_Init(void);//初始化

void ads1220_int_start(void);
void ads1220_int_stop(void);
	

/****************************** end *********************************/






		 				    
#endif
