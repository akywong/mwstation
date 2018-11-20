


#include "adc.h"
#include "delay.h"




/*************************** 文件说明 *******************************/
//此程序适用于七彩光子太极M3—STM32F103开发板及部分兼容的STM32系统，
//淘宝搜索"七彩光子 太极M3 STM32开发板"购买超高性价比开发板或获取更多资料
//本程序部分资源来自网络,只供学习使用，未经作者许可，不得用于其它任何用途！
/********************************************************************/ 
volatile uint16_t AD_value[4];


//初始化ADC
//这里我们仅以规则通道为例
//我们默认将开启通道0~3
void ADC1_Init(void)
{ 	

	
	ADC_InitTypeDef ADC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC |RCC_APB2Periph_ADC1	, ENABLE );	  //使能ADC1通道时钟
 

	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //设置ADC分频因子6 72M/6=12,ADC最大时间不能超过14M

	//PA1 作为模拟通道输入引脚                         
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//模拟输入引脚
	GPIO_Init(GPIOC, &GPIO_InitStructure);	

	ADC_DeInit(ADC1);  //复位ADC1,将外设 ADC1 的全部寄存器重设为缺省值

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC工作模式:ADC1和ADC2工作在独立模式
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;	//模数转换工作在单通道模式
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;	//模数转换工作在单次转换模式
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//转换由软件而不是外部触发启动
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = 4;	//顺序进行规则转换的ADC通道的数目
	ADC_Init(ADC1, &ADC_InitStructure);	//根据ADC_InitStruct中指定的参数初始化外设ADCx的寄存器   

	ADC_RegularChannelConfig(ADC1, 10, 1, ADC_SampleTime_239Cycles5 );	//ADC1,ADC通道,采样时间为239.5周期	  			    
	ADC_RegularChannelConfig(ADC1, 11, 2, ADC_SampleTime_239Cycles5 );
	ADC_RegularChannelConfig(ADC1, 12, 3, ADC_SampleTime_239Cycles5 );
	ADC_RegularChannelConfig(ADC1, 13, 4, ADC_SampleTime_239Cycles5 );
	
	ADC_DMACmd(ADC1, ENABLE);
  
	ADC_Cmd(ADC1, ENABLE);	//使能指定的ADC1
	
	ADC_ResetCalibration(ADC1);	//使能复位校准  
	 
	while(ADC_GetResetCalibrationStatus(ADC1));	//等待复位校准结束
	
	ADC_StartCalibration(ADC1);	 //开启AD校准
 
	while(ADC_GetCalibrationStatus(ADC1));	 //等待校准结束
 
//	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//使能指定的ADC1的软件转换启动功能

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

//获得ADC值
//ch:通道值 0~3
u16 Get_Adc(u8 ch){
  	//设置指定ADC的规则组通道，一个序列，采样时间
	
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//使能指定的ADC1的软件转换启动功能	
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//等待转换结束
	return ADC_GetConversionValue(ADC1);	//返回最近一次ADC1规则组的转换结果
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



























