#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "rotary_encoder.h"
#include "pid_controller.h"
#include "driver/ledc.h"
#include "driver/gpio.h"

#define MOTOR_NUM 2


typedef struct
{
    uint8_t motor_id;
    uint8_t io_pwm;
    uint8_t io_positive;
    uint8_t io_negative;
    uint8_t io_encoder_positive; // A通道
    uint8_t io_encoder_negative; // B通道
} motor_config_t;

/**
 * @brief 初始化电机，编码器&PWM
 *
 * @return true
 * @return false
 */
bool motor_init(void);

/**
 * @brief 初始化配置参数
 *
 * @return true
 * @return false
 */
bool motor_config_init(void);

/**
 * @brief 电机控制任务
 *
 * @param param
 */
void motor_task_init();

#endif