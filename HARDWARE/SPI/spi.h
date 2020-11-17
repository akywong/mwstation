/*   spi.h   */

#ifndef __SPI_H
#define __SPI_H


#include "sys.h"



#define SPI1_CS_H() GPIOA->BSRR=GPIO_Pin_4
#define SPI1_CS_L() GPIOA->BRR =GPIO_Pin_4

#define SPI_LPS22HB_CS_H() GPIOG->BSRR =GPIO_Pin_8
#define SPI_LPS22HB_CS_L() GPIOG->BRR  =GPIO_Pin_8

#define SPI_ADS1220_CS_H() GPIOG->BSRR=GPIO_Pin_3
#define SPI_ADS1220_CS_L() GPIOG->BRR =GPIO_Pin_3


#define SPI2_CS_H() GPIOB->BSRR=GPIO_Pin_12
#define SPI2_CS_L() GPIOB->BRR =GPIO_Pin_12







void SPI1_Init(void);			       //初始化SPI1口
void SPI1_SetMode(u8 cpol,u8 cpha);
void SPI1_SetSpeed(u8 SpeedSet);   //设置SPI1速度  		 
u8 SPI1_ReadWriteByte(u8 TxData);  //SPI1总线读写一个字节

void SPI2_Init(void);			        //初始化SPI口
void SPI2_SetMode(u8 cpol,u8 cpha);
void SPI2_SetSpeed(u8 SpeedSet);    //设置SPI速度
u8 SPI2_ReadWriteByte(u8 TxData);   //SPI总线读写一个字节
		 
#endif

