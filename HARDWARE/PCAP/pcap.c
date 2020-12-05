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
#define PCAP_POWERUP_RESET_CMD  0x88
#define PCAP_INITIAL_RESET_CMD  0x8A
#define PCAP_START_CDC_CMD      0x8C      
#define PCAP_TERMINAL_WOTP_CMD  0x84
#define PCAP_START_RDC_CMD      0x8E
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
void PCAP_powerup_reset(void)
{
	PCAP_send_command(PCAP_POWERUP_RESET_CMD);
}
void PCAP_partitial_reset(void)
{
	PCAP_send_command(PCAP_INITIAL_RESET_CMD);
}
void PCAP_start_cdc(void)
{
	PCAP_send_command(PCAP_START_CDC_CMD);
}
void PCAP_stop_write_otp(void)
{
	PCAP_send_command(PCAP_TERMINAL_WOTP_CMD);
}
void PCAP_start_rdc(void)
{
	PCAP_send_command(PCAP_START_RDC_CMD);
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
#define PCAP_CMD_WOTP  0xA000
#define PCAP_CMD_ROTP  0x2000
void PCAP_otp_op(uint16_t op,uint16_t addr,uint8_t *data)
{
	addr &= 0x1fff;
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
void PCAP_otp_read(uint16_t addr,uint8_t *data)
{
	PCAP_otp_op(PCAP_CMD_ROTP,addr,data);
}
void PCAP_otp_write(uint16_t addr,uint8_t *data)
{
	PCAP_otp_op(PCAP_CMD_WOTP,addr,data);
}
void PCAP_config(uint32_t *regs)
{
	int i;
	for(i=0;i<11;i++){
		PCAPWriteRegister(i,regs[i]);
	}
	PCAP_finish_write();
}
void PCAP_read_status(uint32_t *reg)
{
	PCAPReadRegister(8,reg);
}
void PCAP_read_cdc(uint8_t id,uint32_t *regs)
{
	if(id<8){
		PCAPReadRegister(id,regs);
	}else{
		int i;
		for(i=0;i<8;i++){
			PCAPReadRegister(i,regs+i);
		}
	}
}

void PCAP_read_rdc(uint8_t id,uint32_t *regs)
{
	if(id<2){
		PCAPReadRegister(id+0x13,regs);
	}else{
		int i;
		for(i=0;i<2;i++){
			PCAPReadRegister(i,regs+i);
		}
	}
}