#ifndef __LPS22HB_SPI_H_
#define __LPS22HB_SPI_H_		    

#include "sys.h" 

int32_t lps22hb_spi_read(void *handle,u8 addr, u8 *data, uint16_t count);
int32_t lps22hb_spi_write(void *handle,uint8_t addr, uint8_t *data, uint16_t count);
void lps22hb_delay_ms(uint32_t period);

#endif
















