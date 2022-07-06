/**
 * @brief PID库
 * @author 小鱼 (fishros@foxmail.com)
 * @version V1.0.0
 * @date 2022-07-03
 * @copyright 版权所有：FishBot Open Source Organization
 */
#ifndef _PID_H_
#define _PID_H_

#include "freertos/FreeRTOS.h"

typedef struct
{
    int16_t bias;      //偏差
    int16_t dbias;     //偏差微分
    int16_t bias_sum;  //偏差和
    int16_t bias_last; //上一次偏差的值
    int16_t bias_pre;  //上上一次偏差的值
} pid_calculate_t;

typedef struct
{
    int16_t kp;
    int16_t ki;
    int16_t kd;
    int16_t limit;               //输出限制
    int16_t target;              //目标值
    int16_t output;              //目标值
    pid_calculate_t pid_calcute; // 用于计算的结构体
} pid_controller_t;

#define PID_CONTROLLER(kp, ki, kd, limit) \
    {                                     \
        .kp = kp,                         \
        .ki = ki,                         \
        .kd = kd,                         \
        .limit = limit,                   \
    }

/**
 * @brief 更新PID数值
 *
 * @param pid_config PID配置结构体
 * @param target 目标值
 * @param current 当前值
 * @return uint16_t 系统输出
 */
int16_t pid_update(pid_controller_t *pid_config, int16_t target, int16_t current);

#endif // _PID_H_