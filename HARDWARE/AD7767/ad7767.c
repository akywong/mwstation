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

#define  LEVEL_TOGGLE(x)  x^=1

void TIM2_NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void TIM2_Configuration(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
		
	//设置TIM2 CLK为72M
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);
	//TIM_DeInit(TIM2);

	//自动装载寄存器周期的值//改成500就是0.5ms中断一次
	TIM_TimeBaseStructure.TIM_Period=1000; 

	//时钟预分频72-1
	TIM_TimeBaseStructure.TIM_Prescaler= 71;

	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , DISABLE); 
}

void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2,TIM_IT_Update) != RESET){
		LEVEL_TOGGLE(PBout(12));
		TIM_ClearITPendingBit(TIM2 , TIM_FLAG_Update);
	}
} 
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


	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);	
	GPIO_SetBits(GPIOB,GPIO_Pin_12);
	
	TIM2_Configuration(); 
	TIM2_NVIC_Configuration();
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);
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

