#include "lps22hb_spi.h"
#include "delay.h"
#include "usart.h"
#include "spi.h"

/////Ó²¼þSPI
#define lps22hb_spi_init()         SPI1_SetMode(SPI_CPOL_High,SPI_CPHA_1Edge)
#define lps22hb_cs_h()             SPI1_CS_H()
#define lps22hb_cs_l()             SPI1_CS_L()
#define lps22hb_spi_rw_byte(x)     SPI1_ReadWriteByte(x)

int32_t lps22hb_spi_read(void *handle,u8 addr, u8 *data, uint16_t count)
{
	u8 i;
	
	lps22hb_spi_init();
	
	lps22hb_cs_l();
	lps22hb_spi_rw_byte(addr|0x80); 
	for(i=0; i<count; i++){
		data[i] = lps22hb_spi_rw_byte(0xFF);
	}
	
	lps22hb_cs_h();
	
	return 0;
}

int32_t lps22hb_spi_write(void *handle,uint8_t addr, uint8_t *data, uint16_t count)
{
	u8 i;
	
	lps22hb_spi_init();
	lps22hb_cs_l();
	lps22hb_spi_rw_byte(addr&0x7F); 
	for(i=0; i<count; i++){
		lps22hb_spi_rw_byte(data[i]);
	}
	
	lps22hb_cs_h();
	
	return 0;
}


void lps22hb_delay_ms(uint32_t period)
{
	int i,j;
  for(i=0;i<period;i++){
		for(j=0;j<100000;j++) {
			__nop();
		}
	}
}
