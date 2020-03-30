#ifndef __LPS22HB_SPI_H_
#define __LPS22HB_SPI_H_		    

#include "sys.h" 

unsigned char SPI_Write (unsigned char spiHandle, unsigned char *outData, unsigned char *inData, unsigned char length);
void lps22hb_delay_ms(uint32_t period);

#endif
















