/**
 * @brief 文件描述：V1.0.0.220621版本协议实现代码，主要用于根据协议定义帧、进行数据帧的解析与合成。
 * @author 小鱼 (fishros@foxmail.com)
 * @version V1.0.0
 * @date 2022-07-07
 * @copyright 版权所有：FishBot Open Source Organization
 */
#ifndef _PROTO_V1_0_0_220621_H_
#define _PROTO_V1_0_0_220621_H_

#include "freertos/FreeRTOS.h"

#define MAX_MOTOR_NUM 6

typedef enum
{
    DATA_ENCODER = 0x01,  // 0x01-左右轮编码器
    DATA_IMU = 0x02,      // 0x02-IMU传感器
    DATA_SPEED = 0x03,    // 0x03-速度控制数据
    DATA_PID = 0x04,      // 0x04-PID设置
    DATA_VER_INFO = 0x05, // 0x05-版本信息
} data_id_t;

typedef enum
{
    DATA_TO_MASTER = 0x01, // 0X01,反馈数据（底盘向主控）
    DATA_TO_FBMC = 0x02,   // 0X02,命令数据（主控向底盘）
} data_direction_t;

typedef struct
{
   float kp;
   float ki;
   float kd;
} pid_data_t;

typedef struct
{
    uint16_t motor_speed[MAX_MOTOR_NUM];
} motor_speed_ctrl_t;

typedef struct
{
    float g[3];
    float a[3];
} imu_data_t;

// 数据帧定义

void parse_frame_data(char *frame);


bool register_update_callback_fun();

// void get_frame_from_struct(void);



#endif // _PROTO_V1_0_0_220621_H_