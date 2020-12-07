#include "lps22hb_spi.h"
#include "delay.h"
#include "usart.h"
#include "spi.h"
#define LBS22HB_SPI2

/////Ó²¼þSPI
#ifdef LBS22HB_SPI2
#define lps22hb_spi_init()         SPI2_SetMode(SPI_CPOL_High,SPI_CPHA_2Edge)
#else
#define lps22hb_spi_init()         SPI1_SetMode(SPI_CPOL_High,SPI_CPHA_2Edge)
#endif
#define lps22hb_cs_h()             SPI_LPS22HB_CS_H()
#define lps22hb_cs_l()             SPI_LPS22HB_CS_L()
#ifdef LBS22HB_SPI2
#define lps22hb_spi_rw_byte(x)     SPI2_ReadWriteByte(x)
#else
#define lps22hb_spi_rw_byte(x)     SPI1_ReadWriteByte(x)
#endif

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
