/*  main.c  */

#include "sys.h"
#include "stm32f10x_flash.h"
#include "delay.h"
#include "serial_rtx.h"
#include "usart.h"
#include "led.h"
#include "w25qxx.h"
#include "string.h"
#include "ff.h"
#include "sdio_sdcard.h"
#include "rtc.h"
#include "timer.h"
#include "spi.h"
//#include "key.h"
#include "24cxx.h"
#include "iwdg.h"
#include "io.h"
#include "utils.h"
#include "main.h"

float Rref = 3240.0;
float FlashGainCorrection;

float DacErrorCorrection = 0.99945;
float MeasuredGainCodeValue = 7868944.1883;
unsigned char StartConversion;


uint8_t new_file_flag = 0;
uint32_t new_record_count = 0;

uint8_t send_cmd[12] = {0x24,0x30,0x31,0x2C,0x57,0x56,0x3F,0x2A,0x2F,0x2F,0x0D,0x0A};//$01,WV?*//<cr><lf>
uint8_t send_htset_cmd[13] ={0x24,0x30,0x31,0x2C,0x48,0x54,0x39,0x39,0x2A,0x2F,0x2F,0x0D,0x0A};//$01,HT99*//<cr><lf>
uint8_t send_htq_cmd[12]={0x24,0x30,0x31,0x2C,0x48,0x54,0x3F,0x2A,0x2F,0x2F,0x0D,0x0A};//$01,HT?*//<cr><lf>

uint8_t pms_init_cmd[8]={0x7E,0x00,0x00,0x02,0x01,0x03,0xF9,0x7E};
uint8_t pms_start_cmd[]={0x7E,0x00,0x03,0x00,0xFC,0x7E};

uint16_t record_interval[6]={1,1,10,60,600,3600};

struct record_info{
	float temperature;
	float humidity;
	uint32_t sensor_count;
	float pressure;
	uint32_t press_count;
	float PM1pm;
	float PM2_5pm;
	float PM4pm;
	float PM10pm;
	float PM0_5pcm;
	float PM1pcm;
	float PM2_5pcm;
	float PM4pcm;
	float PM10pcm;
	float tps;
	uint32_t pm_count;
	u16 flag;
};
struct record_info record;
struct record_info record_old;
struct record_info record_last;

struct pm_info pm;
struct fs_status cur;
//
struct sys_status status;
struct sys_config config_r,config_t;
//
float pressure;
double temperature;
double humidity;

float ads1220_temperature;

//struct sys_config test_config;
int check_pms_info(uint8_t *str, int len);
void record_file_write(void);
//void record_head(void);
uint32_t check_config(uint8_t *data);
uint32_t check_cmd(uint8_t *data);
//uint8_t check_between(int min, int max, float data);
//uint8_t check_record(struct record_info r);
//uint8_t pack_data(uint8_t *data,struct record_info r);
//void pack_ht_data(void *buf,uint8_t flag);
void record_head(void);

#define UART_DELAY  12
void RS485_send_data(void *buf,uint8_t len);
int main(void)
{
	u8 t=0,r=0;
	
	memset(&cur, 0, sizeof(struct fs_status));
	memset(&status, 0, sizeof(struct sys_status));
	memset(&pm, 0,sizeof(struct pm_info));
	memset(&record, 0, sizeof(record));
	memset(&record_old, 0, sizeof(record_old));
	memset(&record_old, 0, sizeof(record_last));
	memset(&config_r,0,sizeof(config_r));
	memset(&config_t,0,sizeof(config_t));
	
	delay_init();	    //延时函数初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置中断优先级分组为组2：2位抢占优先级，2位响应优先级
	TIM_SetInterval(1,2000);//1ms
	LED_Init();
	//Key_Init();
	IO_Init();
	SPI1_Init();
	AT24CXX_Init();
	
	USART1_Init(115200); //串口1初始化
	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
	//USART3_Init(9600); //串口3初始化
	//USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
	
	
	if(0){
		config_t.head = 0xAA;
		config_t.tail = 0xAA;
		config_t.type = 0x01;
		config_t.freq = 1;
		config_t.heat_flag = 0;
		config_t.crc = CRC_calCrc8((const unsigned char *)(&config_t), sizeof(config_t)-2);
		USART_SendBuf(USART3,(unsigned char*)&config_t,sizeof(config_t));
	}
	
	while(AT24CXX_Check())
	{
		delay_ms(500);
		printf("EEPROM Check Failed!\r\n");
	}
	delay_ms(1000);
	if(usart3_recv_frame_flag && check_config((uint8_t *)usart3_recv)){
			memcpy(&config_r,usart3_recv,sizeof(config_r));
			AT24CXX_Write(0,(u8*)&config_r,sizeof(config_r));
	} else {
			AT24CXX_Read(0,(u8*)&config_r,sizeof(config_r));
			if(0 == check_config((uint8_t *)&config_r)) {
				config_r.freq = 1;
				config_r.head = 0xAA;
				config_r.tail = 0xAA;
				config_r.heat_flag = 0;
				config_r.crc = CRC_calCrc8((const unsigned char *)(&config_r), sizeof(config_r)-2);
			}
	}
	usart1_recv_frame_flag = 0;
	/*if(CONFIG_IO_GET_IN()) {
		while(1) {
			if(usart1_recv_frame_flag) {
				sscanf((char*)usart1_recv, "$%d,%d,%d,%d,%d:%d:%d,%d",&config.baud,
					&config.year,&config.month,&config.date,&config.hour,&config.minute,&config.second,&config.freq);
				usart1_recv_frame_flag = 0;
				if(config.year == 0) {
					status.rtc_flag =0;
				} else {
					status.rtc_flag = 1;
				}
				USART_SendString(USART1,usart1_recv);
				config.head = 0xAA5555AA;
				config.tail = 0xAA5555AA;
				AT24CXX_Write(0,(u8*)&config,sizeof(config));
				//AT24CXX_Read(0,(u8*)&test_config,sizeof(config));
				break;
			}
			delay_ms(50);
		}
	} else {	
		AT24CXX_Read(0,(u8*)&config,sizeof(config));
		if((config.head != 0xAA5555AA) || (config.tail != 0xAA5555AA)){
			config.head = 0xAA5555AA;
			config.tail = 0xAA5555AA;
			config.baud = 9600;
			config.year = 2019;
			config.month = 16;
			config.date = 8;
			config.hour = 0;
			config.minute = 0;
			config.second = 0;
			config.freq = 1;
			AT24CXX_Write(0,(u8*)&config,sizeof(config));
		}
		status.rtc_flag = 1;
	}*/
	//printf("usart2 baud : %d\r\n",config.baud);
	//printf("freq : %d\r\n",config.freq);
	
	/*while(RTC_Init(status.rtc_flag,config.year,config.month,config.date,config.hour,config.minute,config.second)) {*/
	while(RTC_Init(1,2020,1,1,0,0,0)) {
		delay_ms(100);
	}
	
	
	status.cmd_send_flag = 1;
	
//	Start:
	//开始初始化SD卡
	
	while( SD_OK != SD_Init() ){
		delay_ms(30);
		t++;
		if(t>30){
			t=0;
		}
	}
	cur.sd_cap = (u32)(SDCardInfo.CardCapacity/1024/1024);
	
	delay_ms(500);
	
	//开始初始化文件系统
	while(FR_OK != f_mount(&cur.fs,"0:",1)){
		delay_ms(30);
		t++;
		if(t>30){
			t=0;
		}
	}
	
	delay_ms(100);
	
	//开始创建文件
	snprintf(cur.fpath,32,"%04d-%02d-%02d.txt",
													calendar.w_year,calendar.w_month,calendar.w_date);
	r=f_open(&cur.fsrc,cur.fpath,FA_OPEN_ALWAYS|FA_WRITE);
	if(FR_OK != r){
		//printf("\r\n创建文件失败 !\r\n");
		//goto Start;
	}
	//f_stat(cur.fpath, &cur.finfo);
	//f_lseek(&cur.fsrc,cur.finfo.fsize);
	f_lseek(&cur.fsrc,cur.fsrc.fsize);
	cur.line_num = 0;
	cur.file_flag = 1;
	//printf("usart2 baud : %d\r\n",config.baud);
	//printf("freq : %d\r\n",config.freq);
	if(cur.fsrc.fsize == 0){
		LED_ON(LED0);
		record_head();
		LED_OFF(LED0);
	}
	
	
	USART2_Init(115200); //串口2初始化
	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
	//IWDG_Init_2s();
	RS485_send_data(pms_init_cmd,8);
	while(1)
	{
		//IWDG_Feed();
		if(new_file_flag == 1) {
			f_close(&cur.fsrc);
			new_file_flag = 0;
			snprintf(cur.fpath,32,"%04d-%02d-%02d.txt",
													calendar.w_year,calendar.w_month,calendar.w_date);
			r=f_open(&cur.fsrc,cur.fpath,FA_OPEN_ALWAYS|FA_WRITE);
			if(FR_OK != r){
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
		/*if(usart3_recv_frame_flag){
			usart3_recv_frame_flag = 0;
			if(check_config((uint8_t *)usart3_recv)){
				memcpy(&config_r,usart3_recv,sizeof(config_r));
				status.ht_exp= 1;
				AT24CXX_Write(0,(u8*)&config_r,sizeof(config_r));
			}else if(check_cmd((uint8_t *)usart3_recv)){
				uint8_t uart_data[64];
				if(pack_data(uart_data,record_last)){
					USART_SendBuf(USART3,uart_data,25);
				}
			}
		}*/
		//IWDG_Feed();
		//记录空气质量监测信息
		if(usart2_recv_frame_flag) {
				int ret = check_pms_info(usart2_recv, usart2_recv_cnt);
			  if(ret == 1){
					RS485_send_data(pms_start_cmd,6);
				}else if(ret == 0){
					RS485_send_data(pms_init_cmd,8);
				}else{
					record.PM1pm += pm.PM1pm;
					record.PM2_5pm += pm.PM2_5pm;
					record.PM4pm += pm.PM4pm;
					record.PM10pm += pm.PM10pm;
					record.PM0_5pcm += pm.PM0_5pcm;
					record.PM1pcm += pm.PM1pcm;
					record.PM2_5pcm += pm.PM2_5pcm;
					record.PM4pcm += pm.PM4pcm;
					record.PM10pcm += pm.PM10pcm;
					record.tps += pm.tps;
					record.pm_count++;
				}
        usart2_recv_frame_flag = 0;
        usart2_recv_cnt = 0;
        memset(usart2_recv,0,32);
    } 

		//IWDG_Feed();
		//写文件
		if(((new_record_count - status.last_record_tick) >= record_interval[config_r.freq]) || (new_record_count < status.last_record_tick)){
			LED_ON(LED0);
			status.last_record_tick = new_record_count;
			record_file_write();
			LED_OFF(LED0);
		}
		delay_ms(100);
	}
}
//
int check_pms_info(uint8_t *str, int len)
{
    int i;
		uint32_t sval=0;
	  uint8_t chk;
		uint8_t temp_pm[80];
		uint8_t flag=0;
	  uint8_t real_count;

    if(str == NULL || len>64) {
        return 0;
    } 

    if(str[0] != 0x7E || str[len-1] != 0x7E) {
        return 0;
    }
    for(i=1;i<len-2;i++) {
        sval += str[i];
			  if(i>4){
					if(flag==0){
						if(str[i]!=0x7D){
							temp_pm[real_count++]=str[i];
						}else{
							flag =1;
						}
					}else{
						switch(str[i]){
							case 0x5E:
								temp_pm[real_count++] = 0x7E;
							break;
							case 0x5D:
								temp_pm[real_count++] = 0x7D;
							break;
							case 0x31:
								temp_pm[real_count++] = 0x11;
							break;
							case 0x33:
								temp_pm[real_count++] = 0x13;
							break;
							default:
								break;
						}
						flag = 0;
					}
				}
    } 
    sval &= 0xff;
		chk = sval&0xff;
		chk = ~chk;
		if(str[i-2]!=chk){
			return 0;
		}
		if(real_count == 1){
			return 1;
		}else if(real_count == 40){
			memcpy(&pm,temp_pm,40);
			return 2;
		}
		return 0;
}

void record_file_write(void)
{
	//uint8_t valid_flag = 0;
	UINT count;
	u8 ret;
	int len;
	char prefix[128]={0};
	//uint8_t uart_data[64]={0};
	
	if(record.pm_count != 0) {
		record.PM1pm /= ((float)record.pm_count);
		record.PM2_5pm /= ((float)record.pm_count);
		record.PM4pm /= ((float)record.pm_count);
		record.PM10pm /= ((float)record.pm_count);
		record.PM0_5pcm /= ((float)record.pm_count);
		record.PM1pcm /= ((float)record.pm_count);
		record.PM2_5pcm /= ((float)record.pm_count);
		record.PM4pcm /= ((float)record.pm_count);
		record.PM10pcm /= ((float)record.pm_count);
		record.tps /= ((float)record.pm_count);
		record.flag |= 0x3ff<<3;
	}else{
		/*record.wind_speed = record_old.wind_speed;
		record.wind_direction = record_old.wind_direction;*/
	}
	
	len = sprintf(prefix,"\"%4d-%02d-%02d %02d:%02d:%02d\",%6.2f,%6.2f,%6.2f,%6.2f,%6.2f,%6.2f,%6.2f,%6.2f,%6.2f,%6.2f",
											calendar.w_year,calendar.w_month,calendar.w_date,calendar.hour,calendar.min,calendar.sec,
											record.PM1pm,record.PM2_5pm,record.PM4pm,record.PM10pm,record.PM0_5pcm,
											record.PM1pcm,record.PM2_5pcm,record.PM4pcm,record.PM10pcm,record.tps
											);
	
	prefix[len] = 0x0d;
	prefix[len+1] = 0x0a;
	ret = f_write(&cur.fsrc,prefix,len+2,&count);
		if(FR_OK != ret) {
			cur.file_flag = 0;
			cur.line_num = 0;
			f_close(&cur.fsrc);//FR_OK
			return;
		} else {
			cur.line_num++;
			f_sync(&cur.fsrc);
		}

		//memcpy(&record_old, &record, sizeof(struct record_info));
		USART_SendString(USART1,(unsigned char *)prefix);
		/*if(pack_data(uart_data,record)){
			USART_SendBuf(USART3,uart_data,25);
		}*/
		record_old.humidity = record.humidity;
		record_old.temperature = record.temperature;
		record_old.pressure = record.pressure;
		record_old.PM1pm = record.PM1pm;
		record_old.PM2_5pm = record.PM2_5pm;
		record_old.PM4pm = record.PM4pm;
		record_old.PM10pm = record.PM10pm;
		record_old.PM0_5pcm = record.PM0_5pcm;
		record_old.PM1pcm = record.PM1pcm;
		record_old.PM2_5pcm = record.PM2_5pcm;
		record_old.PM4pcm = record.PM4pcm;
		record_old.PM10pcm = record.PM10pcm;
		record_old.tps = record.tps;
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
	
	len += sprintf(str+len,"\"日期时间\",\"序号\",\"PM1.0\",\"PM2.5\",\"PM4.0\",\"PM10\",\"PM0.5\",\"PM1.0\",\"PM2.5\",\"PM4.0\",\"PM10\",\"TPS\",\"空气温度\",\"空气湿度\",\"大气压力\"");
	str[len] = 0x0d;
	str[len+1] = 0x0a;
	len+=2;
	
	/*len += sprintf(str+len,"\"YYYY-MM-DD hh:mn:ss\",\"-\",\"W/m^2\",\"W/m^2\",\"W/m^2\",\"W/m^2\",\"m/s\",\"°\",\"℃\",\"%%\",\"hPa\"");
	str[len] = 0x0d;
	str[len+1] = 0x0a;
	len+=2;*/
	
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

uint32_t check_config(uint8_t *data)
{
	uint8_t crc=0;
	struct sys_config temp_config;
	memcpy(&temp_config,data,sizeof(temp_config));
	crc = CRC_calCrc8((const unsigned char *)data, sizeof(temp_config)-2);
	if((0x11==temp_config.type) && (crc == temp_config.crc) && (0xAA==temp_config.head) &&(0xAA==temp_config.tail)) {
		return 1;
	}else{
		return 0;
	}
}
uint32_t check_cmd(uint8_t *data)
{
	uint8_t crc = 0;
	crc = CRC_calCrc8((const unsigned char *)data, 3);
	if((data[0]==0xAA)&&(data[1]==0x12)&&(crc==data[2])&&(data[3]==0xAA)){
		return 1;
	}else{
		return 0;
	}
}

/*uint8_t pack_data(uint8_t *data,struct record_info r)
{
	uint8_t flag;
	flag = check_record(r);
	if(flag){
		data[0]=0xAA;
		data[1]=0x02;
		memcpy(data+2,&(r.temperature),4);
		memcpy(data+6,&(r.humidity),4);
		memcpy(data+10,&(r.pressure),4);
		//memcpy(data+14,&(r.wind_speed),4);
		//memcpy(data+18,&(r.wind_direction),4);
		data[22] = flag&record.flag;
		data[23] = CRC_calCrc8((const unsigned char *)data, 23);
		data[24]=0xAA;
		return 1;
	}
	return 0;
}
uint8_t check_between(int min, int max, float data)
{
	if((data>=min) && (data<=max)){
		return 1;
	}else{
		return 0;
	}
}*/

/*uint8_t check_record(struct record_info r)
{
	uint8_t flag = 0;
	flag |= check_temperature(r.temperature);
	flag |= check_humidity(r.humidity)<<1;
	flag |= check_pressure(r.pressure)<<2;
	//flag |= check_wind_speed(r.wind_speed)<<3;
	//flag |= check_wind_direction(r.wind_direction)<<4;
	
	return flag;
}*/
void RS485_send_data(void *buf,uint8_t len)
{
	int i;
	uint8_t *p=(uint8_t *)buf;
	for(i=0;i<len;i++){
		while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
		USART_SendData(USART2,p[i]);
	}
	while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
}
