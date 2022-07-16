/**
 * @brief 文件描述：待更新
 * @author 小鱼 (fishros@foxmail.com)
 * @version V1.0.0
 * @date 2022-07-07
 * @copyright 版权所有：FishBot Open Source Organization
 */
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
#include "protocol.h"

#ifndef MAX_MOTOR_NUM
#define MAX_MOTOR_NUM 4
#endif

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
 * @brief 初始化配置参数
 *
 * @param motor_num_
 * @param motor_configs
 * @param pid_configs
 * @return true
 * @return false
 */
bool set_motor_config(uint8_t motor_num_, motor_config_t *motor_configs, pid_ctrl_config_t *pid_configs);

/**
 * @brief 更新PID参数
 * 
 * @param proto_pid_data 
 * @return true 
 * @return false 
 */
uint8_t update_motor_pid_param(proto_pid_data_t *proto_pid_data);

/**
 * @brief 初始化电机，编码器&PWM
 *
 * @return true
 * @return false
 */
bool motor_init(void);

/**
 * @brief 电机控制任务
 *
 * @param param
 */
void motor_task_init();

#endif // _MOTOR_H_