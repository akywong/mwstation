


#include "adc.h"
#include "delay.h"




/*************************** �ļ�˵�� *******************************/
//�˳����������߲ʹ���̫��M3��STM32F103�����弰���ּ��ݵ�STM32ϵͳ��
//�Ա�����"�߲ʹ��� ̫��M3 STM32������"���򳬸��Լ۱ȿ�������ȡ��������
//�����򲿷���Դ��������,ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;��
/********************************************************************/ 
volatile uint16_t AD_value[4];


//��ʼ��ADC
//�������ǽ��Թ���ͨ��Ϊ��
//����Ĭ�Ͻ�����ͨ��0~3
void ADC1_Init(void)
{ 	

	
	ADC_InitTypeDef ADC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC |RCC_APB2Periph_ADC1	, ENABLE );	  //ʹ��ADC1ͨ��ʱ��
 

	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //����ADC��Ƶ����6 72M/6=12,ADC���ʱ�䲻�ܳ���14M

	//PA1 ��Ϊģ��ͨ����������                         
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//ģ����������
	GPIO_Init(GPIOC, &GPIO_InitStructure);	

	ADC_DeInit(ADC1);  //��λADC1,������ ADC1 ��ȫ���Ĵ�������Ϊȱʡֵ

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC����ģʽ:ADC1��ADC2�����ڶ���ģʽ
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;	//ģ��ת�������ڵ�ͨ��ģʽ
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;	//ģ��ת�������ڵ���ת��ģʽ
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//ת��������������ⲿ��������
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC�����Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel = 4;	//˳����й���ת����ADCͨ������Ŀ
	ADC_Init(ADC1, &ADC_InitStructure);	//����ADC_InitStruct��ָ���Ĳ�����ʼ������ADCx�ļĴ���   

	ADC_RegularChannelConfig(ADC1, 10, 1, ADC_SampleTime_239Cycles5 );	//ADC1,ADCͨ��,����ʱ��Ϊ239.5����	  			    
	ADC_RegularChannelConfig(ADC1, 11, 2, ADC_SampleTime_239Cycles5 );
	ADC_RegularChannelConfig(ADC1, 12, 3, ADC_SampleTime_239Cycles5 );
	ADC_RegularChannelConfig(ADC1, 13, 4, ADC_SampleTime_239Cycles5 );
	
	ADC_DMACmd(ADC1, ENABLE);
  
	ADC_Cmd(ADC1, ENABLE);	//ʹ��ָ����ADC1
	
	ADC_ResetCalibration(ADC1);	//ʹ�ܸ�λУ׼  
	 
	while(ADC_GetResetCalibrationStatus(ADC1));	//�ȴ���λУ׼����
	
	ADC_StartCalibration(ADC1);	 //����ADУ׼
 
	while(ADC_GetCalibrationStatus(ADC1));	 //�ȴ�У׼����
 
//	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//ʹ��ָ����ADC1�����ת����������

}			

void ADC_DMA_Init(void)
{

DMA_InitTypeDef DMA_InitStructure;
//NVIC_InitTypeDef NVIC_InitStructure;
	
RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); 
DMA_DeInit(DMA1_Channel1); 
DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&ADC1->DR; 
DMA_InitStructure.DMA_MemoryBaseAddr = (u32)AD_value; 
DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC; 
DMA_InitStructure.DMA_BufferSize = 4; 
DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 
DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; 
DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; 
DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
DMA_InitStructure.DMA_Priority = DMA_Priority_High;
DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; 
DMA_Init(DMA1_Channel1, &DMA_InitStructure); 

	/*DMA_ClearFlag(DMA1_FLAG_GL4); 
	DMA_Cmd(DMA1_Channel1, DISABLE); 
	
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd =ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	
	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);*/ 
}

//���ADCֵ
//ch:ͨ��ֵ 0~3
u16 Get_Adc(u8 ch){
  	//����ָ��ADC�Ĺ�����ͨ����һ�����У�����ʱ��
	
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//ʹ��ָ����ADC1�����ת����������	
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//�ȴ�ת������
	return ADC_GetConversionValue(ADC1);	//�������һ��ADC1�������ת�����
}
//
u16 Get_Adc_Average(u8 ch,u8 times){
	u32 temp_val=0;
	u8 t;
	for(t=0;t<times;t++)	{
		temp_val+=Get_Adc(ch);
		delay_ms(1);
	}
	return temp_val/times;
} 	 



























