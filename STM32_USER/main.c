/*  main.c  */

#include "sys.h"
#include "stm32f10x_flash.h"
#include "delay.h"
#include "serial_rtx.h"
#include "usart.h"
#include "led.h"
//#include "w25qxx.h"
#include "string.h"
#include "ff.h"
#include "sdio_sdcard.h"
#include "rtc.h"
#include "timer.h"
#include "spi.h"
//#include "key.h"
#include "24cxx.h"
//#include "adc.h"
#include "lps22hb.h"
#include "bsp_ads1256.h"
#include "iwdg.h"
#include "hyt939.h"
#include "ADS1220.h"
#include "utils.h"
#include "main.h"

float Rref = 3240.0;
float FlashGainCorrection;

float DacErrorCorrection = 0.99945;
float MeasuredGainCodeValue = 7868944.1883;
unsigned char StartConversion;
unsigned char ReadConversionData = 0;


uint8_t new_file_flag = 0;
uint32_t new_record_count = 0;

uint8_t send_cmd[12] = {0x24,0x30,0x31,0x2C,0x57,0x56,0x3F,0x2A,0x2F,0x2F,0x0D,0x0A};//$01,WV?*//<cr><lf>
uint8_t send_htset_cmd[13] ={0x24,0x30,0x31,0x2C,0x48,0x54,0x39,0x39,0x2A,0x2F,0x2F,0x0D,0x0A};//$01,HT99*//<cr><lf>
uint8_t send_htq_cmd[12]={0x24,0x30,0x31,0x2C,0x48,0x54,0x3F,0x2A,0x2F,0x2F,0x0D,0x0A};//$01,HT?*//<cr><lf>

uint16_t record_interval[6]={1,1,10,60,600,3600};

struct record_info{
	float wind_speed;
	float wind_direction;
	uint32_t wind_count;
	float temperature;
	float humidity;
	uint32_t sensor_count;
	float pressure;
	uint32_t press_count;
	u8 flag;
};
struct record_info record;
struct record_info record_old;
struct record_info record_last;

//
struct sys_status status;
struct sys_config config_r,config_t;
struct wind_info  wind;
struct fs_status cur;
//struct bme280_dev dev;
//struct bme280_data comp_data;
float pressure;
double temperature;
double humidity;

float ads1220_temperature;

//struct sys_config test_config;
char *check_wind_info(char *str, int len);
void record_file_write(void);
void record_head(void);
uint32_t check_config(uint8_t *data);
uint32_t check_cmd(uint8_t *data);
uint8_t check_between(int min, int max, float data);
uint8_t check_wind_speed(float speed);
uint8_t check_wind_direction(float direction);
uint8_t check_temperature(float temp);
uint8_t check_pressure(float pressure);
uint8_t check_humidity(float humidity);
uint8_t check_record(struct record_info r);
uint8_t pack_data(uint8_t *data,struct record_info r);
void pack_ht_data(void *buf,uint8_t flag);

#define CONFIG_GPIO     GPIOA
#define CONFIG_PIN			GPIO_Pin_1
#define CONFIG_IO_RCC_CLK RCC_APB2Periph_GPIOA
#define CONFIG_IO_GET_IN()  (0)//((CONFIG_GPIO->IDR & CONFIG_PIN)?(1):(0))
void config_gpio_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(CONFIG_IO_RCC_CLK, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = CONFIG_PIN;	 	     //端口配置, 推挽输出
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 		     //上拉输入
	GPIO_Init(CONFIG_GPIO, &GPIO_InitStructure);
}

int main(void)
{
	u8 t=0,r=0;
	
	
	volatile static unsigned char tempData[3];
	unsigned char calibrateCount = 0;
	
	memset(&cur, 0, sizeof(struct fs_status));
	memset(&status, 0, sizeof(struct sys_status));
	memset(&wind, 0,sizeof(struct wind_info));
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
	config_gpio_init();
	SPI1_Init();
	AT24CXX_Init();
	
	// Reset the ADS1220
    ADS1220_Reset();

    // Determine the ADS1220 Calibration offset - Short the AIN0 and AIN1 together and measure the results 4 times.
    // The average result will be subtracted from all future measurements.
    Setup_ADS1220 (ADS1220_MUX_SHORTED, ADS1220_OP_MODE_NORMAL,
                   ADS1220_CONVERSION_SINGLE_SHOT, ADS1220_DATA_RATE_20SPS, ADS1220_GAIN_16, ADS1220_USE_PGA,
                   ADS1220_IDAC1_AIN3, ADS1220_IDAC2_AIN2, ADS1220_IDAC_CURRENT_250_UA);//ADS1220_IDAC2_DISABLED
	
	if(lps22hb_init()){
		LED_ON(LED1);
	}
	
	USART1_Init(115200); //串口1初始化
	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
	USART3_Init(9600); //串口3初始化
	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
	
	
	{
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
	
	
	//lps22hb_init(&dev);
	
	HYT939_Measure_Request();
	
	status.cmd_send_flag = 1;
	
	//Start:
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
	
		ReadConversionData = 0;
    ADS1220_Start ();             // Kick off conversion

    // Gather and average 8 readings from the ADS1220
    while (calibrateCount < 8)
    {
        while (!ReadConversionData);   // Wait for Data Ready interrupt
        ReadConversionData = 0;
        ADS1220_Get_Conversion_Data ((unsigned char *)tempData);   // Get the raw data
        ADS1220_Offset_Calibrate_Data ((unsigned char *)tempData);        // Send results to calibration function
        calibrateCount++;

        // Start next calibration reading?
        if (calibrateCount < 8)
            ADS1220_Start ();
    }
	
		// Configure ADS1220 for actual measurements
    Setup_ADS1220 (ADS1220_MUX_AIN0_AIN1, ADS1220_OP_MODE_NORMAL,
                   ADS1220_CONVERSION_CONTINUOUS, ADS1220_DATA_RATE_20SPS, ADS1220_GAIN_16, ADS1220_USE_PGA,
                   ADS1220_IDAC1_AIN3, ADS1220_IDAC2_AIN2, ADS1220_IDAC_CURRENT_250_UA);//ADS1220_IDAC2_DISABLED
	
		delay_ms(50);
    StartConversion = 0;
    ReadConversionData = 0;
    ADS1220_Start ();      // Only one start needed for Continuous Mode
	
	USART2_Init(9600); //串口2初始化
	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
	HYT939_Measure_Request();
	//IWDG_Init_2s();
	IO_OFF(WINDRE);
	IO_ON(WINDDE);
	pack_ht_data(send_htset_cmd,config_r.heat_flag);
	USART_SendBuf(USART2,send_htset_cmd,13);
	status.last_ht_cmd_tick = tick_count;
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
		if(usart3_recv_frame_flag){
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
		}
		//记录风速计信息
		if(usart2_recv_frame_flag) {
				wind.info_str = check_wind_info((char*)usart2_recv, usart2_recv_cnt);
				if(wind.info_str != NULL) {
					if(strstr((char*)usart2_recv,"$WI,WVP=") == (char*)usart2_recv) {
						sscanf(wind.info_str,"%f,%f,%d",&wind.speed, &wind.direction,&wind.status);
						if(check_wind_speed(wind.speed) && check_wind_direction(wind.direction)) {
							record.wind_speed += wind.speed;
							record.wind_direction += wind.direction;
							record_last.wind_speed = wind.speed;
							record_last.wind_direction = wind.direction;
							record.wind_count++;
							status.last_wind_info = tick_count;
						}
					}else if(strstr((char*)usart2_recv,"$WI,HT=") == (char*)usart2_recv){
						sscanf(wind.info_str,"%d,",&wind.ht);
						if(wind.ht == 99){
							wind.ht_flag = 0;
						}else if(wind.ht == FT742_HT_TEMP){
							wind.ht_flag = 1;
						} else {
							wind.ht_flag = 2;
						}
						if(wind.ht_flag != config_r.heat_flag) {
							pack_ht_data(send_htset_cmd,config_r.heat_flag);
							USART_SendBuf(USART2,send_htset_cmd,13);
							status.last_ht_cmd_tick = tick_count;
						}
					}
				}
        usart2_recv_frame_flag = 0;
        usart2_recv_cnt = 0;
        memset(usart2_recv,0,32);
    } 
		if (0 == usart2_recv_flag){
			if(status.cmd_send_flag){
				if(((tick_count - status.last_cmd_tick) > 400) && ((tick_count - status.last_ht_cmd_tick)>200)){
					USART_SendBuf(USART2,send_cmd,12);
          status.last_cmd_tick = tick_count;
        }
			}
			if(((tick_count - status.last_cmd_tick) > 200) && ((status.ht_exp==1) || ((tick_count - status.last_ht_cmd_tick)>5000))){
					status.ht_exp =0;
					USART_SendBuf(USART2,send_htq_cmd,12);
          status.last_ht_cmd_tick = tick_count;
      }
    }
		//记录气压计信息
		if(((tick_count - status.last_press) >400) || (tick_count < status.last_press)) {
			status.last_press = tick_count;
			if(0 == lps22hb_get_pressure(&pressure)){
				if(check_pressure(pressure)) {
					record.pressure += pressure;
					record_last.pressure = pressure;
					record.press_count++;
				}
			}
		}
		
		//记录温湿度计信息
		if(((tick_count - status.last_sensor) >400) || (tick_count < status.last_sensor)) {
			status.last_sensor = tick_count;
			if(0==HYT939_Data_Fetch(&humidity,&temperature)) {
				if(check_humidity(humidity) && check_temperature(temperature)) {
					record.humidity += humidity;
					record.temperature += temperature;
					record_last.humidity = humidity;
					record_last.temperature = temperature;
					record.sensor_count++;
				}
			}
			HYT939_Measure_Request();
		}
		
		ads1220_temperature = ADS1220_Get_Temperature();
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
void pack_ht_data(void *buf,uint8_t flag)
{
	uint8_t temp=99;
	//uint8_t xor_value = 0;
	uint8_t *data=(uint8_t*)buf;
	//int i;
	if(flag!=0) {
		temp = FT742_HT_TEMP;
	}else{
		temp = 99;
	}
	data[6] = 0x30+temp/10;
	data[7] = 0x30+temp%10;
	
	/*for(i=1;i<8;i++) {
		xor_value ^= data[i];
  }
	
	data[9] = 0x30+xor_value/16;
	data[10] = 0x30+xor_value%16;*/
}
void record_file_write(void)
{
	//uint8_t valid_flag = 0;
	UINT count;
	u8 ret;
	int len;
	char prefix[128]={0};
	uint8_t uart_data[64]={0};
	
	if(record.press_count !=0 ){
		//record.humidity /= ((float)record.sensor_count);
		//record.temperature /= ((float)record.sensor_count);
		record.pressure /= ((float)record.press_count);
		record.flag |= 1<<2;
	}else{
		/*record.humidity = record_old.humidity;
		record.temperature = record_old.temperature;
		record.pressure = record_old.pressure;*/
	}
	if(record.sensor_count !=0 ){
		record.humidity /= ((float)record.sensor_count);
		record.temperature /= ((float)record.sensor_count);
		//record.pressure /= ((float)record.sensor_count);
		record.flag |= 3;
	}
	if(record.wind_count != 0) {
		record.wind_speed /= ((float)record.wind_count);
		record.wind_direction /= ((float)record.wind_count);
		record.flag |= 3<<3;
	}else{
		/*record.wind_speed = record_old.wind_speed;
		record.wind_direction = record_old.wind_direction;*/
	}
	len = sprintf(prefix,"\"%4d-%02d-%02d %02d:%02d:%02d\",%6.2f,%6.2f,%7.2f,%6.2f,%7.2f",
											calendar.w_year,calendar.w_month,calendar.w_date,calendar.hour,calendar.min,calendar.sec,
											record.wind_speed,record.wind_direction,
											record.temperature, record.humidity, record.pressure/100.0
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
		if(pack_data(uart_data,record)){
			USART_SendBuf(USART3,uart_data,25);
		}
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

uint8_t pack_data(uint8_t *data,struct record_info r)
{
	uint8_t flag;
	flag = check_record(r);
	if(flag){
		data[0]=0xAA;
		data[1]=0x02;
		memcpy(data+2,&(r.temperature),4);
		memcpy(data+6,&(r.humidity),4);
		memcpy(data+10,&(r.pressure),4);
		memcpy(data+14,&(r.wind_speed),4);
		memcpy(data+18,&(r.wind_direction),4);
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
}
//检测风速
uint8_t check_wind_speed(float speed)
{
	return check_between(0,75,speed);
}
//
uint8_t check_wind_direction(float direction)
{
	return check_between(0,360,direction);
}
//
uint8_t check_temperature(float temp)
{
	return check_between(-40,80,temp);
}
//
uint8_t check_pressure(float pressure)
{
	return check_between(33000,110000,pressure);
}
//
uint8_t check_humidity(float humidity)
{
	return check_between(0,100,humidity);
}
uint8_t check_record(struct record_info r)
{
	uint8_t flag = 0;
	flag |= check_temperature(r.temperature);
	flag |= check_humidity(r.humidity)<<1;
	flag |= check_pressure(r.pressure)<<2;
	flag |= check_wind_speed(r.wind_speed)<<3;
	flag |= check_wind_direction(r.wind_direction)<<4;
	
	return flag;
	
	/*if(check_wind_speed(r.wind_speed) && check_wind_direction(r.wind_direction) && check_pressure(r.pressure) && check_humidity(r.humidity)&&check_temperature(r.temperature)){
		return 1;
	}else{
		return 0;
	}*/
}
