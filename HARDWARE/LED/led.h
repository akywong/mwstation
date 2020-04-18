#ifndef __LED_H_
#define __LED_H_

#include "sys.h"

/***************** LED操作函数宏定义(低电平点亮) *********************/
#define LED_ON(x)      x=0    //打开LED
#define LED_OFF(x)     x=1    //关闭LED
#define LED_TOGGLE(x)  x^=1   //翻转LED

#define  IO_ON(x)      x=1
#define  IO_OFF(x)     x=0
#define  IO_TOGGLE(x)  x^=1
/****************************** end *********************************/



/***************** LED所在GPIO口时钟宏定义 ***************************/
#define LED0_GPIO_RCC_CLK  RCC_APB2Periph_GPIOB//LED0 GPIO RCC时钟
#define LED1_GPIO_RCC_CLK  RCC_APB2Periph_GPIOE//LED1 GPIO RCC时钟	
/****************************** end *********************************/


/********************* LED所在GPIO口宏定义 ***************************/
#define LED0_GPIO  GPIOB
#define LED1_GPIO  GPIOE
#define WINDRE_GPIO GPIOB
#define WINDDE_GPIO GPIOB

#define LED0_PIN   GPIO_Pin_5
#define LED1_PIN   GPIO_Pin_5
#define WINDRE_PIN GPIO_Pin_0
#define WINDDE_PIN GPIO_Pin_1

#define LED0 PBout(5)// PB5
#define LED1 PEout(5)// PE5
#define WINDRE PBout(0)
#define WINDDE PBout(1)


#define LED_GREEN    LED0   //绿色
#define LED_BLUE     LED1   //蓝色
/****************************** end *********************************/


/*************************** 调用说明 *******************************/
//  1、引用led.h头文件
//  2、调用初始化函数
//  3、使用宏定义操作LED
//  示例:
//      LED_ON(LED0);//打开LED0
//      LED_OFF(LED1);//关闭LED1
//      LED_TOGGLE(LED_GREEN);//翻转绿色LED
/****************************** end *********************************/




/************************* 函数声明 *********************************/
void LED_Init(void);//初始化


/****************************** end *********************************/






		 				    
#endif
