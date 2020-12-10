/*
 * ad7767.h
 *
 *  Created on: 7 DEC 2020
 *      Author: Akywong
 */
#ifndef _AD7767_H_
#define _AD7767_H_

#include <stdint.h>

void ad7767_init();
int ad7767_read_data(int *buf);
int ad7767_powerup(void);
void ad7767_powerdown(void);

#endif
