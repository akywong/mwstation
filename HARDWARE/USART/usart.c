//
#include "stdio.h"
#include "string.h"
#include "sys.h"
#include "serial_rtx.h"
#include "usart.h"	

#include "main.h"


/***********************************************************************/
//������������(֧��USART1 USART2 USART3 UART4 UART5)
//�˳����������߲ʹ���̫��M3��STM32F103�����弰���ּ��ݵ�STM32ϵͳ��
//��ֲע�⣬���������е�STM32F103оƬ��֧��5������
//ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;��
/***********************************************************************/	 


#define USE_MICROLIB   0//�Ƿ�ʹ��MicroLIB,Ĭ�ϲ�ʹ��(���ع���)
#define PRINTF_COM     USART1//printf �˿�ѡ��  USART1��USART2��USART3��UART4��UART5


uint8_t usart1_recv[USART_FRAME_MAX_SIZE]={0};
uint32_t usart1_recv_cnt=0;
uint8_t usart1_recv_frame_flag=0;
uint8_t usart1_recv_flag=0;

uint8_t usart2_recv[USART_FRAME_MAX_SIZE]={0};
uint32_t usart2_recv_cnt=0;
uint8_t usart2_recv_frame_flag=0;
uint8_t usart2_recv_flag=0;


//����1��ʼ������
void USART1_Init(u32 bound){
  
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	//ʹ��USART1��GPIOAʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	
  
	//GPIO�˿�����
	//USART1_TX   GPIOA.9��ʼ��
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;        //PA.9
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	 //�����������
  GPIO_Init(GPIOA, &GPIO_InitStructure);           //��ʼ��GPIOA.9
   
  //USART1_RX	  GPIOA.10��ʼ��
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.10  
	

 
  //USART ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

  USART_Init(USART1, &USART_InitStructure);      //��ʼ������1
	
	//Usart1 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
	
	
	USART_ClearFlag(USART1,USART_FLAG_TC); 
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); //�������ڽ����ж�
	USART_Cmd(USART1, ENABLE);   	//ʹ�ܴ���1 
}



								 
//����2��ʼ������
void USART2_Init(u32 bound)
{  
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
 	NVIC_InitTypeDef NVIC_InitStructure;
 
	//ʱ��ʹ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//ʹ��GPIOAʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//ʹ��USART2ʱ��
	
	//GPIO�˿�����
	//TX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;	           //PA2
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	     //��������
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
	//RX
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;             //PA3
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //��������
  GPIO_Init(GPIOA, &GPIO_InitStructure);  

	RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART2,ENABLE);      //��λ����2
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART2,DISABLE);     //ֹͣ��λ

	USART_InitStructure.USART_BaudRate = bound;                  //����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8λ���ݳ���
	USART_InitStructure.USART_StopBits = USART_StopBits_1;       //һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;          ///��żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//�շ�ģʽ

  USART_Init(USART2, &USART_InitStructure);                //��ʼ������
	USART_ClearFlag(USART2,USART_FLAG_TC); 
  
	//�ж�
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn; //ʹ�ܴ���2�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; //��ռ���ȼ�2��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; //�����ȼ�2��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //ʹ���ⲿ�ж�ͨ��
	NVIC_Init(&NVIC_InitStructure); //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���

	
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
  USART_Cmd(USART2, ENABLE);                    //ʹ�ܴ��� 
	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
}


//����1�жϺ���
void USART1_IRQHandler(void)                	//����1�жϷ������
{
	uint8_t Clear=Clear;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{ 
			if(usart1_recv_cnt < USART_FRAME_MAX_SIZE) {
        usart1_recv[usart1_recv_cnt++] = USART1->DR;
      }
			usart1_recv_flag = 1;
	} 
	else if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET) 
	{
		
			if((memcmp("$", usart1_recv, 1) == 0 ) && (usart1_recv[usart1_recv_cnt-2] == 0x0D) &&  (usart1_recv[usart1_recv_cnt-1] == 0x0A)) {
        usart1_recv_frame_flag = 1;
      }
			Clear=USART1->SR;
			Clear=USART1->DR;
      usart1_recv_flag = 0;
			usart1_recv_cnt=0;
	}
}



//����2�жϺ���
void USART2_IRQHandler(void)
{
	uint8_t Clear=Clear;
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{ 
			if(usart2_recv_cnt < 32) {
        usart2_recv[usart2_recv_cnt++] = USART2->DR;
      }
			usart2_recv_flag = 1;
	} 
	else if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET) 
	{
		
			if((memcmp("$", usart2_recv, 1) == 0 ) && (usart2_recv[usart2_recv_cnt-2] == 0x0D) &&  (usart2_recv[usart2_recv_cnt-1] == 0x0A)) {
        usart2_recv_frame_flag = 1;
      }
			Clear=USART2->SR;
			Clear=USART2->DR;
       usart2_recv_flag = 0;
	}
} 

//����3�жϺ���
void USART3_IRQHandler(void)
{
}

//����4�жϺ���
void UART4_IRQHandler(void)
{
}

//����5�жϺ���
void UART5_IRQHandler(void)
{
}

//���ڷ���һ���ֽ�
void USART_SendByte(USART_TypeDef *USART_COM,u8 c){
}


//���ڷ����ַ�������
void USART_SendString(USART_TypeDef *USART_COM,unsigned char *s)
{
	while(*s)
	{
		while((USART_COM->SR&0X40)==0);//ѭ������,ֱ���������
    USART_COM->DR = (u8)(*s);
		while((USART_COM->SR&0X40)==0);//ѭ������,ֱ���������
		s++;
	}
}

//���ڷ������麯��
void USART_SendBuf(USART_TypeDef *USART_COM,unsigned char *buf,u16 len){
	while(len--){
		while((USART_COM->SR&0X40)==0);//ѭ������,ֱ���������
    USART_COM->DR = (u8)(*buf++);
		while((USART_COM->SR&0X40)==0);//ѭ������,ֱ���������
	}
}



/********************* printf ʵ�� ****************************/

#if USE_MICROLIB!=1
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{
	while((PRINTF_COM->SR&0X40)==0);//ѭ������,ֱ���������   
    PRINTF_COM->DR = (u8) ch;      
	return ch;
}
#else
/*ʹ��microLib�ķ���*/

int fputc(int ch, FILE *f)
{
	USART_SendData(PRINTF_COM, (uint8_t) ch);
	while (USART_GetFlagStatus(PRINTF_COM, USART_FLAG_TC) == RESET) {}	
  return ch;
}
int GetKey (void){ 
    while (!(PRINTF_COM->SR & USART_FLAG_RXNE));
    return ((int)(PRINTF_COM->DR & 0x1FF));
}

#endif 
 
/****************************** end *********************************/















































 
 
// 
//#if EN_USART1_RX   //���ʹ���˽���
////����1�жϷ������
////ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���   	
//u8 USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
////����״̬
////bit15��	������ɱ�־
////bit14��	���յ�0x0d
////bit13~0��	���յ�����Ч�ֽ���Ŀ
//u16 USART_RX_STA=0;       //����״̬���	  
//  
//void uart_init(u32 bound){
//  //GPIO�˿�����
//  GPIO_InitTypeDef GPIO_InitStructure;
//	USART_InitTypeDef USART_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
//	 
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//ʹ��USART1��GPIOAʱ��
//  
//	//USART1_TX   GPIOA.9
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
//  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.9
//   
//  //USART1_RX	  GPIOA.10��ʼ��
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
//  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.10  

//  //Usart1 NVIC ����
//  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//��ռ���ȼ�3
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//�����ȼ�3
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
//	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
//  
//   //USART ��ʼ������

//	USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
//	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
//	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
//	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

//  USART_Init(USART1, &USART_InitStructure); //��ʼ������1
//  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
//  USART_Cmd(USART1, ENABLE);                    //ʹ�ܴ���1 

//}

//void USART1_IRQHandler(void)                	//����1�жϷ������
//	{
//	u8 Res;
//#if SYSTEM_SUPPORT_OS 		//���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
//	OSIntEnter();    
//#endif
//	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
//		{
//		Res =USART_ReceiveData(USART1);	//��ȡ���յ�������
//		
//		if((USART_RX_STA&0x8000)==0)//����δ���
//			{
//			if(USART_RX_STA&0x4000)//���յ���0x0d
//				{
//				if(Res!=0x0a)USART_RX_STA=0;//���մ���,���¿�ʼ
//				else USART_RX_STA|=0x8000;	//��������� 
//				}
//			else //��û�յ�0X0D
//				{	
//				if(Res==0x0d)USART_RX_STA|=0x4000;
//				else
//					{
//					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
//					USART_RX_STA++;
//					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//�������ݴ���,���¿�ʼ����	  
//					}		 
//				}
//			}   		 
//     } 
//#if SYSTEM_SUPPORT_OS 	//���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
//	OSIntExit();  											 
//#endif
//} 
//#endif	

