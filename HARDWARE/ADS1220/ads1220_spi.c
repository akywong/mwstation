#include "ads1220_spi.h"
#include "delay.h"
#include "usart.h"
#include "spi.h"

/////Ó²¼þSPI
#define ads1220_spi_init()         SPI1_SetMode(SPI_CPOL_Low,SPI_CPHA_2Edge)
#define ads1220_cs_h()             SPI1_ADS1220_CS_H()
#define ads1220_cs_l()             SPI1_ADS1220_CS_L()
#define ads1220_spi_rw_byte(x)     SPI1_ReadWriteByte(x)


unsigned char SPI_Write (unsigned char spiHandle, unsigned char *outData, unsigned char *inData, unsigned char length)
{
	u8 i;
	
	ads1220_spi_init();
	ads1220_cs_l();
	for(i=0; i<length; i++){
		inData[i] = ads1220_spi_rw_byte(outData[i]);
	}
	ads1220_cs_h();
	
	return 0;
}

void ads1220_delay_ms(uint32_t period)
{
	int i,j;
  for(i=0;i<period;i++){
		for(j=0;j<100000;j++) {
			__nop();
		}
	}
}
