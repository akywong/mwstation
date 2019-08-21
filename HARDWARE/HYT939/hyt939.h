/*   hyt939.h   */

#ifndef __HYT939_H_
#define __HYT939_H_

#include "sys.h" 

void HYT939_Init(void);
u8 HYT939_Measure_Request(void);
u8 HYT939_Data_Fetch(double *hum,double *temp);

#endif
