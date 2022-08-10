#ifndef __NVS_H__
#define __NVS_H__
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp32_i2c_rw.h"

void nvs_test(void);

void nvs_read_string(char * string_name);
void nvs_read_struct(char * struct_name,void * pStruct,uint32_t length);

void nvs_write_string(char * string_name,char * string_value);
void nvs_write_struct(char * struct_name,void * pStruct,uint32_t length);

/*写 */
void nvs_write_uint8(char * i8_name,int8_t i8);

/*读取int */
void nvs_read_uint8(char * i8_name,int8_t *i8);


#endif