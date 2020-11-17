/*     iwdg.c    */


#include "iwdg.h"

/*
功能:初始化独立看门狗
prer:分频数:0~7(只有低3位有效!)分频因子=4*2^prer.但最大值只能是256!
rlr :重装载寄存器值:低11位有效.
说明:时间计算(大概):Tout=((4*2^prer)*rlr)/40 (ms).
*/
void IWDG_Init(u8 prer,u16 rlr) 
{	
#ifdef SET_WATCHDOG
 	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);  //使能对寄存器IWDG_PR和IWDG_RLR的写操作
	IWDG_SetPrescaler(prer);  //设置IWDG预分频值:设置IWDG预分频值为64
	IWDG_SetReload(rlr);  //设置IWDG重装载值
	IWDG_ReloadCounter();  //按照IWDG重装载寄存器的值重装载IWDG计数器
	IWDG_Enable();  //使能IWDG
#endif
	return;
}
//
void IWDG_Init_10ms(void){
	IWDG_Init(IWDG_Prescaler_4,100);
}
//
void IWDG_Init_50ms(void){
	IWDG_Init(IWDG_Prescaler_4,500);
}
//
void IWDG_Init_200ms(void){
	IWDG_Init(IWDG_Prescaler_4,2000);
}
//
void IWDG_Init_500ms(void){
	IWDG_Init(IWDG_Prescaler_8,2500);
}
//
void IWDG_Init_1s(void){
	IWDG_Init(IWDG_Prescaler_64,625);
}
//
void IWDG_Init_2s(void){
	IWDG_Init(IWDG_Prescaler_64,1250);
}
//
void IWDG_Init_4s(void){
	IWDG_Init(IWDG_Prescaler_64,2500);
}
//
void IWDG_Init_8s(void){
	IWDG_Init(IWDG_Prescaler_128,2500);
}
//
void IWDG_Init_16s(void){
	IWDG_Init(IWDG_Prescaler_256,2500);
}

//喂独立看门狗
void IWDG_Feed(void)
{   
#ifdef SET_WATCHDOG
 	IWDG_ReloadCounter();//reload		
#endif
	return;
}


