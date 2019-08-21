#include "hyt939.h" 
#include "delay.h"
#include "ioiic.h"


#define HYT939_IIC_Init()         IO_IIC_Init()
#define HYT939_IIC_Start()        IO_IIC_Start()
#define HYT939_IIC_Stop()         IO_IIC_Stop()
#define HYT939_IIC_Write_Byte(x)  IO_IIC_Write_Byte(x)
#define HYT939_IIC_Read_Byte(x)   IO_IIC_Read_Byte(x)
#define HYT939_IIC_Wait_Ack()     IO_IIC_Wait_Ack()

#define SLAVLE_ADDR 0x28

//初始化IIC接口
void HYT939_Init(void)
{
	HYT939_IIC_Init();
}
//发送一个转换请求
u8 HYT939_Measure_Request(void)
{				  
	u8 ret = 0;		  	    																 
  HYT939_IIC_Start();
  HYT939_IIC_Write_Byte((SLAVLE_ADDR<<1) | 0);
	ret = HYT939_IIC_Wait_Ack();
	if(ret != 0) {
		return ret;
	}
	HYT939_IIC_Stop();
	return 0;
}
	
	
//接收转换数据
u8 HYT939_Data_Fetch(double *hum,double *temp)
{
	u8 ret=0;
	u8 data[4]={0};
	int i;
	u16 _hum,_temp;
	
	HYT939_IIC_Start();  
	HYT939_IIC_Write_Byte((SLAVLE_ADDR<<1) | 1);	    //发送写命令
	ret = HYT939_IIC_Wait_Ack();
	if (ret != 0) {
		return ret;
	}
	
	for (i=0; i<3; i++) {
		data[i] = HYT939_IIC_Read_Byte(1);
	}
	data[3] = HYT939_IIC_Read_Byte(0);
	
	_hum = ((data[0]&0x3f)<<8) | data[1];
	_temp = ((data[2]<<8) | data[3])>>2;
	
	*hum = (double)_hum*100.0/16383.0;//(double)(1<<14-1);
	*temp = (165.0/16383.0) * (double)_temp -40.0;
	
	HYT939_IIC_Stop();
	return 0;
}
