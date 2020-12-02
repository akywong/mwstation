/*   spi.c   */

#include "spi.h"

//SPI1��ʼ��
void SPI1_Init(void)
{
 	GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;
	 
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA|RCC_APB2Periph_SPI1|RCC_APB2Periph_GPIOC, ENABLE );//PORTA��SPI1ʱ��ʹ�� 

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;                 //PA5.6.7��������  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

 	GPIO_SetBits(GPIOA,GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7); //PA5.6.7����
	
	//CS�ܽų�ʼ��  PA4
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA, ENABLE );//PORTAʱ��ʹ�� 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;        // PB12 ���� 
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOA, &GPIO_InitStructure);
 	GPIO_SetBits(GPIOA,GPIO_Pin_4);
	
	//CS�ܽų�ʼ��  PC0
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOC, ENABLE );//PORTCʱ��ʹ�� 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;        // PB12 ���� 
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOC, &GPIO_InitStructure);
 	GPIO_SetBits(GPIOC,GPIO_Pin_0);
	
	//CS�ܽų�ʼ��  PD12
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOD, ENABLE );//PORTDʱ��ʹ�� 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;        // PD12 ���� 
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOD, &GPIO_InitStructure);
 	GPIO_SetBits(GPIOD,GPIO_Pin_12);
	
			
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //����SPI�������˫�������ģʽ:SPI����Ϊ˫��˫��ȫ˫��
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		        //����SPI����ģʽ:����Ϊ��SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		    //����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		          //ѡ���˴���ʱ�ӵ���̬:ʱ�����ո�
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	          //���ݲ����ڵڶ���ʱ����
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;	            	//NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;		//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ16
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	    //ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
	SPI_InitStructure.SPI_CRCPolynomial = 7;	              //CRCֵ����Ķ���ʽ
	SPI_Init(SPI1, &SPI_InitStructure);                     //����SPI_InitStruct��ָ���Ĳ�����ʼ������SPIx�Ĵ���
 
	SPI_Cmd(SPI1, ENABLE); //ʹ��SPI����
	
	SPI1_ReadWriteByte(0xff);//��������	
}


void SPI1_SetMode(u8 cpol,u8 cpha)
{
	SPI_InitTypeDef  SPI_InitStructure;
	
	GPIO_SetBits(GPIOA,GPIO_Pin_4);//lps22hbƬѡ����
	GPIO_SetBits(GPIOC,GPIO_Pin_0);//ads1220Ƭѡ����
	SPI_Cmd(SPI1, DISABLE); //
	
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //����SPI�������˫�������ģʽ:SPI����Ϊ˫��˫��ȫ˫��
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		        //����SPI����ģʽ:����Ϊ��SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		    //����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ
	SPI_InitStructure.SPI_CPOL = cpol;		          //ѡ���˴���ʱ�ӵ���̬:ʱ�����ո�
	SPI_InitStructure.SPI_CPHA = cpha;	          //���ݲ����ڵڶ���ʱ����
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;	            	//NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;		//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ16
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	    //ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
	SPI_InitStructure.SPI_CRCPolynomial = 7;	              //CRCֵ����Ķ���ʽ
	SPI_Init(SPI1, &SPI_InitStructure);                     //����SPI_InitStruct��ָ���Ĳ�����ʼ������SPIx�Ĵ���
	
	SPI_Cmd(SPI1, ENABLE); //ʹ��SPI����
	
	SPI1_ReadWriteByte(0xff);//��������	
}

void SPI1_SetSpeed(u8 SpeedSet)
{
	assert_param(IS_SPI_BAUDRATE_PRESCALER(SpeedSet));
	SPI1->CR1&=0XFFC7; 
	SPI1->CR1|=SpeedSet;
	SPI_Cmd(SPI1,ENABLE); 
}


//SPIx ��дһ���ֽ�
//TxData:Ҫд����ֽ�
//����ֵ:��ȡ�����ֽ�
u8 SPI1_ReadWriteByte(u8 TxData)
{				   			 
	u8 retry=0;				 	
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) //���ָ����SPI��־λ�������:���ͻ���ձ�־λ
	{
		retry++;
		if(retry>200)return 0;
	}
	SPI_I2S_SendData(SPI1, TxData); //ͨ������SPIx����һ������
	retry=0;
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET) //���ָ����SPI��־λ�������:���ܻ���ǿձ�־λ
	{
		retry++;
		if(retry>200)return 0;
	}	  						    
	return SPI_I2S_ReceiveData(SPI1); //����ͨ��SPIx������յ�����					    
}

//SPI2��ʼ��
void SPI2_Init(void)
{
 	GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;
	 
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOG|RCC_APB2Periph_GPIOD|RCC_APB2Periph_GPIOB, ENABLE );//PORTB,PORTD��SPI1ʱ��ʹ�� 
	RCC_APB1PeriphClockCmd(	RCC_APB1Periph_SPI2, ENABLE );
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;                 //PD13.14.15��������  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

 	GPIO_SetBits(GPIOB,GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15); 
	
	//ads1248Ƭѡ
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOD, ENABLE );//PORTDʱ��ʹ�� 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;        // PD12 ���� 
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOD, &GPIO_InitStructure);
 	GPIO_SetBits(GPIOD,GPIO_Pin_12);
	//as3935Ƭѡ
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOD, ENABLE );//PORTDʱ��ʹ�� 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;        // PD12 ���� 
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOD, &GPIO_InitStructure);
 	GPIO_SetBits(GPIOD,GPIO_Pin_0);
	
	//ads1220Ƭѡ
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOG, ENABLE );//PORTGʱ��ʹ�� 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;        // PG3���� 
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOG, &GPIO_InitStructure);
 	GPIO_SetBits(GPIOG,GPIO_Pin_3);
	
	//lps22hbƬѡ
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOG, ENABLE );//PORTDʱ��ʹ�� 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;        // PG8 ���� 
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOG, &GPIO_InitStructure);
 	GPIO_SetBits(GPIOG,GPIO_Pin_8);
	
	//ADS1256Ƭѡ
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOE, ENABLE );//PORTDʱ��ʹ�� 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;        // PE8 ���� 
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOE, &GPIO_InitStructure);
 	GPIO_SetBits(GPIOE,GPIO_Pin_8);
	
			
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //����SPI�������˫�������ģʽ:SPI����Ϊ˫��˫��ȫ˫��
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		        //����SPI����ģʽ:����Ϊ��SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		    //����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		          //ѡ���˴���ʱ�ӵ���̬:ʱ�����ո�
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	          //���ݲ����ڵڶ���ʱ����
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;	            	//NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;		//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ16
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	    //ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
	SPI_InitStructure.SPI_CRCPolynomial = 7;	              //CRCֵ����Ķ���ʽ
	SPI_Init(SPI2, &SPI_InitStructure);                     //����SPI_InitStruct��ָ���Ĳ�����ʼ������SPIx�Ĵ���
 
	SPI_Cmd(SPI2, ENABLE); //ʹ��SPI����
	
	SPI2_ReadWriteByte(0xff);//��������	
}


void SPI2_SetMode(u8 cpol,u8 cpha)
{
	SPI_InitTypeDef  SPI_InitStructure;
	
	GPIO_SetBits(GPIOD,GPIO_Pin_12);//ads1248Ƭѡ����
	SPI_Cmd(SPI2, DISABLE); //
	
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //����SPI�������˫�������ģʽ:SPI����Ϊ˫��˫��ȫ˫��
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		        //����SPI����ģʽ:����Ϊ��SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		    //����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ
	SPI_InitStructure.SPI_CPOL = cpol;		          //ѡ���˴���ʱ�ӵ���̬:ʱ�����ո�
	SPI_InitStructure.SPI_CPHA = cpha;	          //���ݲ����ڵڶ���ʱ����
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;	            	//NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;		//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ16
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	    //ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
	SPI_InitStructure.SPI_CRCPolynomial = 7;	              //CRCֵ����Ķ���ʽ
	SPI_Init(SPI2, &SPI_InitStructure);                     //����SPI_InitStruct��ָ���Ĳ�����ʼ������SPIx�Ĵ���
	
	SPI_Cmd(SPI2, ENABLE); //ʹ��SPI����
	
	SPI2_ReadWriteByte(0xff);//��������	
}

void SPI2_SetSpeed(u8 SpeedSet)
{
	assert_param(IS_SPI_BAUDRATE_PRESCALER(SpeedSet));
	SPI2->CR1&=0XFFC7; 
	SPI2->CR1|=SpeedSet;
	SPI_Cmd(SPI2,ENABLE); 
}


//SPIx ��дһ���ֽ�
//TxData:Ҫд����ֽ�
//����ֵ:��ȡ�����ֽ�
u8 SPI2_ReadWriteByte(u8 TxData)
{				   			 
	u8 retry=0;				 	
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET) //���ָ����SPI��־λ�������:���ͻ���ձ�־λ
	{
		retry++;
		if(retry>200)return 0;
	}
	SPI_I2S_SendData(SPI2, TxData); //ͨ������SPIx����һ������
	retry=0;
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET) //���ָ����SPI��־λ�������:���ܻ���ǿձ�־λ
	{
		retry++;
		if(retry>200)return 0;
	}	  						    
	return SPI_I2S_ReceiveData(SPI2); //����ͨ��SPIx������յ�����					    
}






























