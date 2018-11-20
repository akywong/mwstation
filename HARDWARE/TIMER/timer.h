#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"

extern uint32_t tick_count;

void TIM_SetInterval(u8 TIM_Index,u32 nus);
void TIM_Init(u8 TIM_Index,u16 Prescaler,u16 Period);

#endif
