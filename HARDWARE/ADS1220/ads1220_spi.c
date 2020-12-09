#include "ads1220_spi.h"
#include "delay.h"
#include "usart.h"
#include "spi.h"


#define ADS1220_SPI2

/////Ó²¼þSPI
#ifdef ADS1220_SPI2
#define ads1220_spi_init()         SPI2_SetMode(SPI_CPOL_High,SPI_CPHA_2Edge)
#else
#define ads1220_spi_init()         SPI1_SetMode(SPI_CPOL_High,SPI_CPHA_2Edge)
#endif
#define ads1220_cs_h()             SPI_ADS1220_CS_H()
#define ads1220_cs_l()             SPI_ADS1220_CS_L()
#ifdef ADS1220_SPI2
#define ads1220_spi_rw_byte(x)     SPI2_ReadWriteByte(x)
#else
#define ads1220_spi_rw_byte(x)     SPI1_ReadWriteByte(x)
#endif

unsigned char SPI_Write (unsigned char spiHandle, unsigned char *outData, unsigned char *inData, unsigned char length)
{
	u8 i;
	
	ads1220_spi_init();
	ads1220_cs_l();
	delay_ms(1);
	for(i=0; i<length; i++){
		inData[i] = ads1220_spi_rw_byte(outData[i]);
	}
	delay_ms(1);
	ads1220_cs_h();
	
	return 0;
}

