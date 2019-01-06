#ifndef __USART_H_
#define __USART_H_


#include "stdio.h"
#include "sys.h"

#define USART_FRAME_MAX_SIZE  256

extern uint8_t usart1_recv[USART_FRAME_MAX_SIZE];
extern uint32_t usart1_recv_cnt;
extern uint8_t usart1_recv_frame_flag;
extern uint8_t usart1_recv_flag;

extern uint8_t usart2_recv[USART_FRAME_MAX_SIZE];
extern uint32_t usart2_recv_cnt;
extern uint8_t usart2_recv_frame_flag;
extern uint8_t usart2_recv_flag;


void USART1_Init(u32 bound);//����1��ʼ������
void USART2_Init(u32 bound);//����2��ʼ������
void USART3_Init(u32 bound);//����3��ʼ������
void UART4_Init (u32 bound);//����4��ʼ������
void UART5_Init (u32 bound);//����5��ʼ������
//���ڷ���һ���ֽ�
void USART_SendByte(USART_TypeDef *USART_COM,u8 c);
//���ڷ������麯��
void USART_SendBuf(USART_TypeDef *USART_COM,unsigned char *buf,u16 len);
//���ڷ����ַ�������
void USART_SendString(USART_TypeDef *USART_COM,unsigned char *s);



#endif


