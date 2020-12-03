/*
 * pcap.h
 *
 * Created on: 7 Nov 2020
 * Author: akywong
 */
#ifndef _PCAP_H_
#define _PCAP_H_

#include <stdint.h>

#define PCAP_CS_PIN        	 GPIO_Pin_6
#define PCAP_CS_GPIO_PORT  	 GPIOG
#define PCAP_CS_GPIO_CLK   	 RCC_APB2Periph_GPIOG

void PCAP_sram_read(uint16_t addr,uint8_t *data);
void PCAP_sram_write(uint16_t addr,uint8_t *data);
#endif /* _PCAP_H_ */
