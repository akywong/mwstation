#include "bme_spi.h"
#include "delay.h"
#include "usart.h"
#include "spi.h"

/////Ó²¼þSPI
#define bme_spi_init()         SPI1_Init()
#define bme_cs_h()             SPI1_CS_H()
#define bme_cs_l()             SPI1_CS_L()
#define bme_spi_rw_byte(x)     SPI1_ReadWriteByte(x)

int8_t bme_read_reg(uint8_t id, u8 addr, u8 *data, uint16_t count)
{
	u8 i;
	
	bme_cs_l();
	bme_spi_rw_byte(addr|0x80); 
	for(i=0; i<count; i++){
		data[i] = bme_spi_rw_byte(0xFF);
	}
	
	bme_cs_h();
	
	return 0;
}

int8_t bme_write_reg(uint8_t id, uint8_t addr, uint8_t *data, uint16_t count)
{
	u8 i;
	
	bme_cs_l();
	bme_spi_rw_byte(addr&0x7F); 
	for(i=0; i<count; i++){
		bme_spi_rw_byte(data[i]);
	}
	
	bme_cs_h();
	
	return 0;
}


void bme_delay_ms(uint32_t period)
{
	int i,j;
  for(i=0;i<period;i++){
		for(j=0;j<100000;j++) {
			__nop();
		}
	}
}
