	/*
	 * pcap.c
	 * Author: Aky
	 */
#include "spi.h"
#include "pcap.h"
#include "delay.h"

#define PCAP_SPI2

#ifdef PCAP_SPI2
#define pcap_spi_init()         SPI2_SetMode(SPI_CPOL_Low,SPI_CPHA_2Edge)
#else
#define pcap_spi_init()         SPI1_SetMode(SPI_CPOL_Low,SPI_CPHA_2Edge)
#endif
#define PCAP_DELAY(n)   delay_ms(n)

#define PCAP_CMD_WREG         0xC0
#define PCAP_CMD_RREG	        0x40
#define PCAP_DISABLE()        GPIO_SetBits(PCAP_CS_GPIO_PORT, PCAP_CS_PIN)
#define PCAP_ENABLE()         GPIO_ResetBits(PCAP_CS_GPIO_PORT, PCAP_CS_PIN)
	
void pcap_io_init(void)
{
	/*GPIO_InitTypeDef  GPIO_InitStructure; 
	
	RCC_APB2PeriphClockCmd(PCAP_CS_GPIO_CLK,ENABLE);  	 

	GPIO_InitStructure.GPIO_Pin = PCAP_CS_PIN;		
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(PCAP_CS_GPIO_PORT, &GPIO_InitStructure);	
	
	GPIO_SetBits(PCAP_CS_GPIO_PORT,PCAP_CS_PIN);*/
}

static unsigned char PCAP_SPI_SendByte(unsigned char byte){
#ifdef PCAP_SPI2
	return SPI2_ReadWriteByte(byte);
#else
	return SPI1_ReadWriteByte(byte);
#endif
}

  
void PCAPReadRegister(uint8_t addr, uint32_t *data)
{
	int i;
	uint8_t buf[3];
	 
	pcap_spi_init();
	// assert CS to start transfer 
	PCAP_ENABLE();  
	PCAP_DELAY(1);
	// send the command byte
	PCAP_SPI_SendByte(PCAP_CMD_RREG | (addr & 0x3f));
	// get the register content
	for (i=0; i< 3; i++)
	{
		buf[i] = PCAP_SPI_SendByte(0xFF);
	} 
	
	*data = buf[2]|(buf[1]<<8)|(buf[0]<<16);
	
	PCAP_DELAY(1);
	PCAP_DISABLE(); 
	
	return;
}

void PCAPWriteRegister(uint8_t addr, uint32_t data)
{
	int i;
	uint8_t buf[3];
	
	buf[0]= (data>>16)&0xFF;
	buf[1]= (data>>8)&0xFF;
	buf[2]=  data&0xFF;
	
	pcap_spi_init();
	// set the CS low  
	PCAP_ENABLE(); 
	PCAP_DELAY(1);
	// send the command byte
	PCAP_SPI_SendByte(PCAP_CMD_WREG | (addr & 0x3f));
	// send the data bytes
	for (i=0; i < 3; i++)
	{
		PCAP_SPI_SendByte(buf[i]);
	} 
	
	PCAP_DELAY(1); 
	PCAP_DISABLE();
}

void PCAP_finish_write()
{
	PCAPWriteRegister(0x14,1);
}

void PCAP_send_command(uint8_t command)
{
	pcap_spi_init();
	// set the CS low  
	PCAP_ENABLE(); 
	PCAP_DELAY(1);
	// send the command byte
	PCAP_SPI_SendByte(command);
	PCAP_DELAY(1); 
	PCAP_DISABLE();
}
#define PCAP_CMD_WSRAM 0x9000
#define PCAP_CMD_RSRAM 0x1000
void PCAP_sram_op(uint16_t op,uint16_t addr,uint8_t *data)
{
	addr &= 0x0fff;
	addr |= op;
	
	pcap_spi_init();
	// set the CS low  
	PCAP_ENABLE(); 
	PCAP_DELAY(1);
	// send the command byte
	PCAP_SPI_SendByte((addr>>8)&0xff);
	PCAP_SPI_SendByte(addr&0xff);
	// send the data bytes
	PCAP_SPI_SendByte(*data);
	
	PCAP_DELAY(1); 
	PCAP_DISABLE();
}
void PCAP_sram_read(uint16_t addr,uint8_t *data)
{
	PCAP_sram_op(PCAP_CMD_RSRAM,addr,data);
}
void PCAP_sram_write(uint16_t addr,uint8_t *data)
{
	PCAP_sram_op(PCAP_CMD_WSRAM,addr,data);
}