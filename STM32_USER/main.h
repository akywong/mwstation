#ifndef __MAIN_H__
#define __MAIN_H__
#include "ff.h"
#define WIND_INFO_UNVALID 0x01
#define BME_SENSOR_UNVALID 0x02
#define ADC_0_UNVALID 0x04
#define ADC_1_UNVALID 0x08
#define ADC_2_UNVALID 0x10
#define ADC_3_UNVALID 0x20

struct fs_status{
	FATFS fs;
	FIL   fsrc;
	char  fpath[32];
	FILINFO finfo;
	uint8_t file_flag;
	uint32_t sd_cap;
	uint32_t file_id;
	uint32_t line_num;
};

struct sys_status{
		uint32_t sys_config_flag;
		uint32_t last_record_tick;
    uint32_t last_cmd_tick;
		uint32_t last_wind_info;
		uint32_t last_adc;
		uint32_t last_sensor;//ÎÂÊª¶È¼Æ
    uint8_t cmd_send_flag;
    uint8_t lora_send_flag;
		uint8_t stop_feed_flag;
};

struct wind_info{
	char  *info_str;
	float speed;
	float direction;
	char status;
	uint8_t flag;
};

struct sys_config{
	uint32_t baud;
	float cal_A;
	float cal_B;
	uint32_t year;
	uint32_t month;
	uint32_t date;
	uint32_t hour;
	uint32_t minute;
	uint32_t second;
	uint32_t freq;
};

extern uint8_t new_file_flag;
#endif
