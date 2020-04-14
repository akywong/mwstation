#include "io.h"
#include "main.h"
    
//GPIO IO��ʼ��
void IO_Init(void)
{
 GPIO_InitTypeDef  GPIO_InitStructure;
 uint32_t peri;
	
	peri = CONFIG_IO_RCC_CLK|ADS1220_DRY_RCC_CLK|LPS22HB_DRY_RCC_CLK;
	RCC_APB2PeriphClockCmd(peri, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = CONFIG_PIN;	 	     //�˿�����, �������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 		     //��������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(CONFIG_GPIO, &GPIO_InitStructure);
	
	//
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	
	GPIO_InitStructure.GPIO_Pin = ADS1220_DRY_PIN;
	GPIO_Init(ADS1220_DRY_GPIO, &GPIO_InitStructure);
}
 
void ads1220_int_start(void)
{
	EXTI_InitTypeDef   EXTI_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;

	/* Enable AFIO clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	/* Connect EXTI0 Line to PC0 pin */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource1);

	/* Configure EXTI0 line */
	EXTI_InitStructure.EXTI_Line = EXTI_Line1;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  /* �½��� */
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Enable and set EXTI0 Interrupt  priority */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void ads1220_int_stop(void)
{
	EXTI_InitTypeDef   EXTI_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;

	/* ���� EXTI LineXXX */
	EXTI_InitStructure.EXTI_Line = EXTI_Line1;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;	/* �½��� */
	EXTI_InitStructure.EXTI_LineCmd = DISABLE;		/* ��ֹ */
	EXTI_Init(&EXTI_InitStructure);

	/* �ж����ȼ����� ������ȼ� ����һ��Ҫ�ֿ��������жϣ����ܹ��ϲ���һ���������� */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;
	NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;		/* ��ֹ */
	NVIC_Init(&NVIC_InitStructure);
}
unsigned char ReadConversionData = 0;
void EXTI1_IRQHandler(void)
{
	ReadConversionData=1;
}



