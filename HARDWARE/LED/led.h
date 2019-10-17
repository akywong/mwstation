#ifndef __LED_H_
#define __LED_H_

#include "sys.h"

/***************** LED���������궨��(�͵�ƽ����) *********************/
#define LED_ON(x)      x=0    //��LED
#define LED_OFF(x)     x=1    //�ر�LED
#define LED_TOGGLE(x)  x^=1   //��תLED

#define  IO_ON(x)      x=1
#define  IO_OFF(x)     x=0
#define  IO_TOGGLE(x)  x^=1
/****************************** end *********************************/



/***************** LED����GPIO��ʱ�Ӻ궨�� ***************************/
#define LED0_GPIO_RCC_CLK  RCC_APB2Periph_GPIOB//LED0 GPIO RCCʱ��
#define LED1_GPIO_RCC_CLK  RCC_APB2Periph_GPIOE//LED1 GPIO RCCʱ��	
/****************************** end *********************************/


/********************* LED����GPIO�ں궨�� ***************************/
#define LED0_GPIO  GPIOB
#define LED1_GPIO  GPIOE
#define WINDRE_GPIO GPIOB
#define WINDDE_GPIO GPIOB

#define LED0_PIN   GPIO_Pin_5
#define LED1_PIN   GPIO_Pin_5
#define WINDRE_PIN GPIO_Pin_2
#define WINDDE_PIN GPIO_Pin_1

#define LED0 PBout(5)// PB5
#define LED1 PEout(5)// PE5
#define WINDRE PBout(2)
#define WINDDE PBout(1)


#define LED_GREEN    LED0   //��ɫ
#define LED_BLUE     LED1   //��ɫ
/****************************** end *********************************/


/*************************** ����˵�� *******************************/
//  1������led.hͷ�ļ�
//  2�����ó�ʼ������
//  3��ʹ�ú궨�����LED
//  ʾ��:
//      LED_ON(LED0);//��LED0
//      LED_OFF(LED1);//�ر�LED1
//      LED_TOGGLE(LED_GREEN);//��ת��ɫLED
/****************************** end *********************************/




/************************* �������� *********************************/
void LED_Init(void);//��ʼ��


/****************************** end *********************************/






		 				    
#endif
