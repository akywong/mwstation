#ifndef __BME_SPI_H_
#define __BME_SPI_H_		    

#include "sys.h" 

int8_t bme_read_reg(uint8_t id, u8 addr, u8 *data, uint16_t count);
int8_t bme_write_reg(uint8_t id, uint8_t addr, uint8_t *data, uint16_t count);
void bme_delay_ms(uint32_t period);

#endif
















