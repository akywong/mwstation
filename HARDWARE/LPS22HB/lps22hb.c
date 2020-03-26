/**\mainpage
 * Copyright (C) 2016 - 2017 Bosch Sensortec GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holder nor the names of the
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 *
 * The information provided is believed to be accurate and reliable.
 * The copyright holder assumes no responsibility
 * for the consequences of use
 * of such information nor for any infringement of patents or
 * other rights of third parties which may result from its use.
 * No license is granted by implication or otherwise under any patent or
 * patent rights of the copyright holder.
 *
 * File		lps22hb.c
 * Date		14 Feb 2018
 * Version	3.3.4
 *
 */

/*! @file lps22hb.c
    @brief Sensor driver for lps22hb sensor */
#include <string.h>
#include "spi.h"
#include "lps22hb_spi.h"
#include "lps22hb_reg.h"
#include "lps22hb.h"


stmdev_ctx_t dev_ctx;

u8 lps22hb_init(void)
{
	u8 whoamI,rst;
	u32 trycount = 999;
	/* Initialize mems driver interface */
  dev_ctx.write_reg = lps22hb_spi_write;
  dev_ctx.read_reg = lps22hb_spi_read;
  //dev_ctx.handle = null;
	
	lps22hb_device_id_get(&dev_ctx, &whoamI);
  if (whoamI != LPS22HB_ID){
    //printf("read lps22hb chip id failed!\n");
		return 1;
  }
	
	/* Restore default configuration */
  lps22hb_reset_set(&dev_ctx, PROPERTY_ENABLE);
  do {
    lps22hb_reset_get(&dev_ctx, &rst);
		trycount--;
		if(trycount==0){
			return 2;
		}
  } while (rst);
	
	/* Can be enabled low pass filter on output */
  lps22hb_low_pass_filter_mode_set(&dev_ctx, LPS22HB_LPF_ODR_DIV_2);

  /* Can be set Data-ready signal on INT_DRDY pin */
  //lps22hb_drdy_on_int_set(&dev_ctx, PROPERTY_ENABLE);

  /* Set Output Data Rate */
  lps22hb_data_rate_set(&dev_ctx, LPS22HB_ODR_10_Hz);
	
	return 0;
}

/*typedef union{
  int16_t i16bit;
  uint8_t u8bit[2];
} axis1bit16_t;*/

typedef union{
  int32_t i32bit;
  uint8_t u8bit[4];
} axis1bit32_t;
int lps22hb_get_pressure(float *pressure)
{
		uint8_t reg=0;
		axis1bit32_t data_raw_pressure;
		//axis1bit16_t data_raw_temperature;
		//float temperature_degC;
   
    /* Read output only if new value is available */
		
    lps22hb_press_data_ready_get(&dev_ctx, &reg);
    if (reg)
    {
      memset(data_raw_pressure.u8bit, 0x00, sizeof(int32_t));
      lps22hb_pressure_raw_get(&dev_ctx, data_raw_pressure.u8bit);
     
      *pressure = lps22hb_from_lsb_to_hpa(data_raw_pressure.i32bit);
      //sprintf((char*)tx_buffer, "pressure [hPa]:%6.2f\r\n", pressure_hPa);

      /*tx_com(tx_buffer, strlen((char const*)tx_buffer));

      memset(data_raw_temperature.u8bit, 0x00, sizeof(int16_t));
      lps22hb_temperature_raw_get(&dev_ctx, data_raw_temperature.u8bit);

      temperature_degC = lps22hb_from_lsb_to_degc(data_raw_temperature.i16bit);
      sprintf((char*)tx_buffer, "temperature [degC]:%6.2f\r\n", temperature_degC);

      tx_com(tx_buffer, strlen((char const*)tx_buffer));*/
			return 0;
    }else{
			return 1;
		}
	}
