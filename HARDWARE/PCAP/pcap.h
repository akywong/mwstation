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

void PCAP_powerup_reset(void);
void PCAP_partitial_reset(void);
void PCAP_start_cdc(void);
void PCAP_stop_write_otp(void);
void PCAP_start_rdc(void);
void PCAP_sram_read(uint16_t addr,uint8_t *data);
void PCAP_sram_write(uint16_t addr,uint8_t data);
void PCAP_config(uint32_t *regs);
void PCAP_read_status(uint32_t *reg);
void PCAP_read_cdc(uint8_t id,uint32_t *regs);
void PCAP_read_rdc(uint8_t id,uint32_t *regs);
#endif /* _PCAP_H_ */
