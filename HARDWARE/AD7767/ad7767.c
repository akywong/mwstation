/*
 * ad7767.c
 * Author: Aky
 */
 
#include "spi.h"
#include "ad7767.h"
#include "delay.h"


//#define AD7767_SPI2

#ifdef AD7767_SPI2
#define ad7767_spi_init()         SPI2_SetMode(SPI_CPOL_Low,SPI_CPHA_2Edge)
#else
#define ad7767_spi_init()         SPI1_SetMode(SPI_CPOL_Low,SPI_CPHA_2Edge)
#endif
#define AD7767_DELAY(n)   				delay_ms(n)

/*#define AD7767_CS_PIN        	 GPIO_Pin_0
#define AD7767_CS_GPIO_PORT  	 GPIOD
#define AD7767_CS_GPIO_CLK   	 RCC_APB2Periph_GPIOD*/

#define AD7767_DRDY_PIN        	 GPIO_Pin_8
#define AD7767_DRDY_GPIO_PORT  	 GPIOD
#define AD7767_DRDY_GPIO_CLK   	 RCC_APB2Periph_GPIOD

#define AD7767_POWERDOWN_PIN        	 GPIO_Pin_9
#define AD7767_POWERDOWN_GPIO_PORT  	 GPIOD
#define AD7767_POWERDOWN_GPIO_CLK   	 RCC_APB2Periph_GPIOD

#define AD7767_DRDY GPIO_ReadInputDataBit(AD7767_DRDY_GPIO_PORT, AD7767_DRDY_PIN)
void ad7767_init()
{
	GPIO_InitTypeDef  GPIO_InitStructure; 
	
	RCC_APB2PeriphClockCmd(AD7767_DRDY_GPIO_CLK|AD7767_POWERDOWN_GPIO_CLK,ENABLE);  	 

	GPIO_InitStructure.GPIO_Pin = AD7767_DRDY_PIN;		
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(AD7767_DRDY_GPIO_PORT, &GPIO_InitStructure);	
	
	GPIO_InitStructure.GPIO_Pin = AD7767_POWERDOWN_PIN;		
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(AD7767_POWERDOWN_GPIO_PORT, &GPIO_InitStructure);	
}

static unsigned char AD7767_SPI_SendByte(unsigned char byte){
#ifdef AD7767_SPI2
	return SPI2_ReadWriteByte(byte);
#else
	return SPI1_ReadWriteByte(byte);
#endif
}
int ad7767_read_data(int *buf)
{
	int i;
	int timeout=1000;
	char check=1;
	unsigned int data = 0x00000000;
	unsigned int wbuf[3] = {0xAA,0xAA,0xAA};
	unsigned int rbuf[3] = {0x00,0x00,0x00};
	
	while(timeout&&check){
			check = AD7767_DRDY;
			timeout--;
	}
	
	ad7767_spi_init();
	// assert CS to start transfer 
	//AD7767_ENABLE();  
	AD7767_DELAY(1);
	for (i=0; i< 3; i++){
		rbuf[i] = AD7767_SPI_SendByte(wbuf[i]);
	} 
	AD7767_DELAY(1);
	//AD7767_DISABLE(); 
	
	data = rbuf[0];
	data <<= 8;
	data |= rbuf[1];
	data <<= 8;
  data |= rbuf[2];
  data &= 0x00FFFFFF;
	*buf = data;
	//return(data);
	if(timeout)
		return 1;
	else
		return 0;
}


	
/**
		@brief: The below function returns the 24bit data sent serially by the AD7767.
**/


void ad7767_powerdown(void)                       // Enter power down mode
{
	GPIO_ResetBits(AD7767_POWERDOWN_GPIO_PORT,AD7767_POWERDOWN_PIN);
}

int ad7767_powerup(void)                          // Wake up from power down
{
	int check;
	int timeout=1000;
	GPIO_SetBits(AD7767_POWERDOWN_GPIO_PORT,AD7767_POWERDOWN_PIN);
	check = AD7767_DRDY;
	while(timeout&&check)
	{
		timeout--;
		AD7767_DELAY(1);
		check = AD7767_DRDY;
	}
	if(timeout){
		return 1;
	}
	return 0;
}	

