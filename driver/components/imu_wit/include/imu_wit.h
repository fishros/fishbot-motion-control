/*
 * @作者: 小鱼
 * @公众号: 鱼香ROS
 * @QQ交流群: 2642868461
 * @描述: IMU头文件
 */
#pragma once
#ifdef __cplusplus
extern "C" {
#endif
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "udp_client.h"


typedef struct 
{
    uint8_t START;
    uint8_t TARGET;
    uint8_t CODE;
	uint8_t data_len;
    short sacc[3]; 
    short sgy[3]; 
    short sangle[3];
    uint8_t sum;
    uint8_t END;
}__attribute__ ((packed)) imu_t;



static void uart_init();


void my_imu_init();

#ifdef __cplusplus
}
#endif