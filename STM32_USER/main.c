/*  main.c  */

#include "sys.h"
#include "delay.h"
#include "serial_rtx.h"
#include "usart.h"
#include "led.h"
//#include "w25qxx.h"
#include "string.h"
//#include "ff.h"
#include "sdio_sdcard.h"
#include "beep.h"
#include "rtc.h"
#include "timer.h"
#include "key.h"
//#include "adc.h"
#include "bme280.h"
#include "bsp_ads1256.h"
#include "main.h"

uint8_t new_file_flag = 0;

uint8_t send_cmd[12] = {0x24,0x30,0x31,0x2C,0x57,0x56,0x3F,0x2A,0x2F,0x2F,0x0D,0x0A};//$01,WV?*//

#define ADC_CHAN_NUM 4
double ADC_value[ADC_CHAN_NUM];

//
struct sys_status status;
struct sys_config config;
struct wind_info  wind;
struct fs_status cur;
struct bme280_dev dev;
struct bme280_data comp_data;
char *check_wind_info(char *str, int len);
void record_file_write(void);
void record_head(void);

struct record_info{
	double wind_speed;
	double wind_direction;
	uint32_t wind_count;
	double temperature;
	double humidity;
	double pressure;
	uint32_t sensor_count;
	double ADC_value0;
	double ADC_value1;
	double ADC_value2;
	double ADC_value3;
	uint32_t ADC_count;
};
struct record_info record;
struct record_info record_old;

int main(void)
{
	u8 t=0,r=0;
	memset(&cur, 0, sizeof(struct fs_status));
	memset(&status, 0, sizeof(struct sys_status));
	memset(&wind, 0,sizeof(struct wind_info));
	memset(ADC_value,0,sizeof(double)*ADC_CHAN_NUM);
	memset(&record, 0, sizeof(record));
	memset(&record_old, 0, sizeof(record_old));
	delay_init();	    //延时函数初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置中断优先级分组为组2：2位抢占优先级，2位响应优先级
	TIM_SetInterval(1,2000);//1ms
	LED_Init();
	Beep_Init();
	Key_Init();
	
	USART1_Init(115200); //串口1初始化
	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
	
	//while(status.sys_config_flag == 0){
	LED_ON(LED1);
	config.baud = 9600;
	config.cal_A = 1.0;
	config.cal_B = 0;
	/*config.year = 0;
	config.month = 11;
	config.date = 7;
	config.hour = 14;
	config.minute = 30;
	config.second = 30;*/
	config.freq = 1;
	config.rtc_flag = 0;
	config.ad_gain = ADS1256_GAIN_1;
	do{
		if(usart1_recv_frame_flag) {
				sscanf((char*)usart1_recv, "$%d,%f,%f,%d,%d,%d,%d:%d:%d,%d,%d",&config.baud,&config.cal_A,&config.cal_B,
					&config.year,&config.month,&config.date,&config.hour,&config.minute,&config.second,&config.ad_gain,&config.freq);
				if(config.year !=0){
					config.rtc_flag = 1;
				}else{
					config.rtc_flag = 1;
				}
				usart1_recv_frame_flag = 0;
				status.sys_config_flag =1;
				USART_SendString(USART1,usart1_recv);
				break;
		} else {
			if(KEY0_HARD_STA | KEY1_HARD_STA | KEY2_HARD_STA | KEY3_HARD_STA){
				status.sys_config_flag =1;
				break;
			}
		}
	}while(1);
	
	LED_OFF(LED1);
	/*while(1){
		delay_ms(500);
		LED_TOGGLE(LED1);
	}*/
	while(RTC_Init(config.rtc_flag,config.year,config.month,config.date,config.hour,config.minute,config.second)) {
		delay_ms(100);
	}
	
	bme280_init(&dev);
	
	bsp_InitADS1256();
	
	ADS1256_CfgADC((ADS1256_GAIN_E)config.ad_gain, ADS1256_15SPS);
	ADS1256_StartScan(1);	
	
	status.cmd_send_flag = 1;
	
	//Start:
	//开始初始化SD卡
	while( SD_OK != SD_Init() ){
		delay_ms(30);
		t++;
		if(t%10==0){
			BEEP_TOGGLE();
			LED_TOGGLE(LED1);
		}
		if(t>30){
			t=0;
		}
	}
	BEEP_OFF();
	cur.sd_cap = (u32)(SDCardInfo.CardCapacity/1024/1024);
	
	delay_ms(1000);
	
	//开始初始化文件系统
	while(FR_OK != f_mount(&cur.fs,"0:",1)){
		delay_ms(30);
		t++;
		if(t%10==0){
			BEEP_TOGGLE();
			LED_TOGGLE(LED1);
		}
		if(t>30){
			t=0;
		}
	}
	BEEP_OFF();
	
	delay_ms(100);
	
	//开始创建文件
	snprintf(cur.fpath,32,"%04d-%02d-%02d.txt",
													calendar.w_year,calendar.w_month,calendar.w_date);
	r=f_open(&cur.fsrc,cur.fpath,FA_OPEN_ALWAYS|FA_WRITE);
	if(FR_OK != r){
		//printf("\r\n创建文件失败 !\r\n");
		//goto Start;
		BEEP_ON();
	}
	//f_stat(cur.fpath, &cur.finfo);
	//f_lseek(&cur.fsrc,cur.finfo.fsize);
	f_lseek(&cur.fsrc,cur.fsrc.fsize);
	cur.line_num = 0;
	cur.file_flag = 1;
	
	if(cur.fsrc.fsize == 0){
		LED_ON(LED0);
		record_head();
		LED_OFF(LED0);
	}
	
	USART2_Init(config.baud); //串口2初始化
	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
	if(status.cmd_send_flag ) {
				USART_SendBuf(USART2,send_cmd,12);
        status.last_cmd_tick = tick_count;
    }
	LED_ON(LED1);
	while(1)
	{
		if(new_file_flag == 1) {
			f_close(&cur.fsrc);
			new_file_flag = 0;
			snprintf(cur.fpath,32,"%04d-%02d-%02d.txt",
													calendar.w_year,calendar.w_month,calendar.w_date);
			r=f_open(&cur.fsrc,cur.fpath,FA_OPEN_ALWAYS|FA_WRITE);
			if(FR_OK != r){
				BEEP_ON();
			}else {
				BEEP_OFF();
			}
			//f_stat(cur.fpath, &cur.finfo);
			//f_lseek(&cur.fsrc,cur.finfo.fsize);
			//printf("\r\n创建文件成功 !\r\n");
			f_lseek(&cur.fsrc,cur.fsrc.fsize);
			cur.line_num = 0;
			cur.file_flag = 1;
			if(cur.fsrc.fsize == 0){
				record_head();
			}
		}
		//记录风速计信息
		if(usart2_recv_frame_flag) {
				wind.info_str = check_wind_info((char*)usart2_recv, usart2_recv_cnt);
				if(wind.info_str != NULL) {
					sscanf(wind.info_str,"%f,%f,%c",&wind.speed, &wind.direction,&wind.status);
					record.wind_speed += wind.speed;
					record.wind_direction += wind.direction;
					record.wind_count++;
					wind.flag = 1;
					status.last_wind_info = tick_count;
				}
        usart2_recv_frame_flag = 0;
        usart2_recv_cnt = 0;
        memset(usart2_recv,0,32);
    } 
		if (0 == usart2_recv_flag){
			if(status.cmd_send_flag){
				if((tick_count - status.last_cmd_tick) > 400){
					USART_SendBuf(USART2,send_cmd,12);
          status.last_cmd_tick = tick_count;
        }
			} 
    }
		//记录温湿度计信息
		if((tick_count - status.last_sensor) >400) {
			status.last_sensor = tick_count;
			if(BME280_OK == bme280_get_sensor_data(BME280_ALL, &comp_data, &dev)){
				record.humidity += comp_data.humidity;
				record.temperature += comp_data.temperature;
				record.pressure += comp_data.pressure;
				record.sensor_count++;
			}
		}
		//记录AD转换信息
		if((tick_count - status.last_adc) > 400) {
			status.last_adc = tick_count;
			record.ADC_value0 += (double)ADS1256_GetAdc(0);
			record.ADC_value1 += (double)ADS1256_GetAdc(1);
			record.ADC_value2 += (double)ADS1256_GetAdc(2);
			record.ADC_value3 += (double)ADS1256_GetAdc(3);
			record.ADC_count++;
		}
		
		//写文件
		if((tick_count - status.last_record_tick) > (500*config.freq)){
			LED_ON(LED0);
			status.last_record_tick = tick_count;
			record_file_write();
			LED_OFF(LED0);
		}
	}
}
//检查风速计信息格式
char *check_wind_info(char *str, int len)
{
    int i;
    int xor_value = 0;
    int value ;
    int equal_sign=0;
    int asterisk =0;

    if(str == NULL || len>31) {
        return NULL;
    } 

    if(*str != '$' || str[len-2] != 0x0d || str[len-1] != 0x0a) {
        return NULL;
    }
    for(i=1;i<len-2;i++) {
        if(str[i] == '=') {
            equal_sign = i;
        }else if(str[i] == '*') {
            asterisk = i;
            break;
        }
        xor_value ^= str[i];
    } 
    
    if(equal_sign ==0 || asterisk == 0) {
        return NULL;
    }
    
    sscanf(str+i+1,"%x",&value);
    if(xor_value != value) {
        return NULL;
    }
    
    return (str+equal_sign+1);
}

void record_file_write(void)
{
	//uint8_t valid_flag = 0;
	UINT count;
	u8 ret;
	int len;
	char prefix[128];
	/*if(tick_count-status.last_wind_info>1500) {
		valid_flag |= WIND_INFO_UNVALID;
	}
	if(BME280_OK != bme280_get_sensor_data(BME280_ALL, &comp_data, &dev)){
		valid_flag |= BME_SENSOR_UNVALID;
	}*/
	if(record.ADC_count !=0 ){
		record.ADC_value0 = (record.ADC_value0/(double)record.ADC_count)*config.cal_A+config.cal_B;
		record.ADC_value1 = (record.ADC_value1/(double)record.ADC_count)*config.cal_A+config.cal_B;
		record.ADC_value2 = (record.ADC_value2/(double)record.ADC_count)*config.cal_A+config.cal_B;
		record.ADC_value3 = (record.ADC_value3/(double)record.ADC_count)*config.cal_A+config.cal_B;
		
		record.ADC_value0 = (record.ADC_value0 * 2.5000000) / 4194303.0/(double)(1<<config.ad_gain);
		record.ADC_value1 = (record.ADC_value1 * 2.5000000) / 4194303.0/(double)(1<<config.ad_gain);
		record.ADC_value2 = (record.ADC_value2 * 2.5000000) / 4194303.0/(double)(1<<config.ad_gain);
		record.ADC_value3 = (record.ADC_value3 * 2.5000000) / 4194303.0/(double)(1<<config.ad_gain);
	}else{
		//record.ADC_value0 = record_old.ADC_value0;
		//record.ADC_value1 = record_old.ADC_value1;
		//record.ADC_value2 = record_old.ADC_value2;
		//record.ADC_value3 = record_old.ADC_value3;
	}
	
	if(record.sensor_count !=0 ){
		record.humidity /= ((double)record.sensor_count);
		record.temperature /= ((double)record.sensor_count);
		record.pressure /= ((double)record.sensor_count);
	}else{
		//record.humidity = record_old.humidity;
		//record.temperature = record_old.temperature;
		//record.pressure = record_old.pressure;
	}
	if(record.wind_count != 0) {
		record.wind_speed /= ((double)record.wind_count);
		record.wind_direction /= ((double)record.wind_count);
	}else{
		//record.wind_speed = record_old.wind_speed;
		//record.wind_direction = record_old.wind_direction;
	}
	len = sprintf(prefix,"\"%4d-%02d-%02d %02d:%02d:%02d\",%5d,%8.6f,%8.6f,%8.6f,%8.6f,%6.2f,%6.2f,%7.2f,%6.2f,%7.2f",
											calendar.w_year,calendar.w_month,calendar.w_date,calendar.hour,calendar.min,calendar.sec,
											cur.line_num,record.ADC_value0,record.ADC_value1,record.ADC_value2,record.ADC_value3,
											record.wind_speed,record.wind_direction,
											record.temperature, record.humidity, record.pressure/100.0
											);
	
	prefix[len] = 0x0d;
	prefix[len+1] = 0x0a;
	ret = f_write(&cur.fsrc,prefix,len+2,&count);
		//LED_TOGGLE(LED1);
		if(FR_OK != ret) {
			cur.file_flag = 0;
			cur.line_num = 0;
			f_close(&cur.fsrc);
			return;
		} else {
			cur.line_num++;
			f_sync(&cur.fsrc);
		}
		/*if((cur.line_num * config.freq) > 10) {
			f_sync(&cur.fsrc);
		}*/
		//memcpy(&record_old, &record, sizeof(struct record_info));
		record_old.ADC_value0 = record.ADC_value0;
		record_old.ADC_value1 = record.ADC_value1;
		record_old.ADC_value2 = record.ADC_value2;
		record_old.ADC_value3 = record.ADC_value3;
		record_old.humidity = record.humidity;
		record_old.temperature = record.temperature;
		record_old.pressure = record.pressure;
		record_old.wind_speed = record.wind_speed;
		record_old.wind_direction = record.wind_direction;
		memset(&record, 0, sizeof(record));
}

void record_head(void)
{
	int len;
	char str[256];
	int ret;
	UINT count;
	
	len = sprintf(str,"\"TOPFLAG\",\"小型气象站\",\"版本V1.0\",\"数据平均\"");
	str[len] = 0x0d;
	str[len+1] = 0x0a;
	len+=2;
	
	len += sprintf(str+len,"\"日期时间\",\"序号\",\"总辐射\",\"直接辐射\",\"散射辐射\",\"倾斜辐射\",\"风速\",\"风向\",\"空气温度\",\"空气湿度\",\"大气压力\"");
	str[len] = 0x0d;
	str[len+1] = 0x0a;
	len+=2;
	
	len += sprintf(str+len,"\"YYYY-MM-DD hh:mn:ss\",\"-\",\"W/m^2\",\"W/m^2\",\"W/m^2\",\"W/m^2\",\"m/s\",\"°\",\"℃\",\"%%\",\"hPa\"");
	str[len] = 0x0d;
	str[len+1] = 0x0a;
	len+=2;
	
	ret = f_write(&cur.fsrc,str,len,&count);
	if(FR_OK != ret) {
		cur.file_flag = 0;
		cur.line_num = 0;
		f_close(&cur.fsrc);
		return;
	} else {
		//cur.line_num++;
		f_sync(&cur.fsrc);
	}
}

void get_ADC_value(void)
{
	int32_t adc[ADC_CHAN_NUM];
	int i;
	
	for(i=0; i<ADC_CHAN_NUM; i++)
	{
		adc[i] = ADS1256_GetAdc(i);
		ADC_value[i] = ((double)adc[i] * 2500000.0) / 4194303.0;
	}
}
