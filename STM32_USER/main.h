#ifndef __MAIN_H__
#define __MAIN_H__
#include "ff.h"
#define WIND_INFO_UNVALID 0x01
#define BME_SENSOR_UNVALID 0x02

#define FT742_HT_TEMP  5

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
		uint32_t last_ht_cmd_tick;
		uint32_t last_wind_info;
		uint32_t last_sensor;//ÎÂÊª¶È¼Æ
    uint32_t last_press;
		uint32_t last_adc;//Ì«Ñô·øÉä
		uint8_t cmd_send_flag;
    uint8_t lora_send_flag;
		uint8_t stop_feed_flag;
		uint8_t rtc_flag;
		uint8_t ht_exp;
};

struct wind_info{
	char  *info_str;
	float speed;
	float direction;
	int   ht;
	//char status;
	int status;
	uint8_t ht_flag;
};

struct sys_config{
	uint8_t head;
	uint8_t type;
	uint8_t freq;
	uint8_t ad_gain;
	uint8_t heat_flag;
	uint8_t crc;
	uint8_t tail;
};

extern uint8_t new_file_flag;
extern uint32_t new_record_count;
#endif
