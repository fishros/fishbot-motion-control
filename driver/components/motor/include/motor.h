/*
 * @作者: 小鱼
 * @公众号: 鱼香ROS
 * @QQ交流群: 2642868461
 * @描述: 电机头文件
 */
#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "rotary_encoder.h"
#include "driver/ledc.h"
#include "driver/gpio.h"


typedef struct 
{
    uint8_t START;
    uint8_t TARGET;
    uint8_t CODE;
    uint8_t data_len; 
    int16_t spped_left;
    int16_t spped_right;
    uint8_t sum;
    uint8_t END;
}__attribute__ ((packed)) motor;


typedef struct 
{
    int16_t bias; //偏差微分
    int16_t dbias; //偏差的
    int16_t bias_sum; //偏差值
    int16_t bias_last;  //上一次偏差的值
    int16_t bias_pre; //上上一次偏差的值
    int16_t output; //输出结果
} my_pid_t;

void my_motor_init();

#ifdef __cplusplus
}
#endif