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
#include "proto_utils.h"

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
} proto_pid_data_t;

typedef struct
{
    uint16_t motor_speed[MAX_MOTOR_NUM];
} motor_speed_ctrl_t;

typedef struct
{
    float accel[3];
    float gyro[3];
    float quat[4];
} imu_data_t;

/* 定义更新PID数据的钩子 */
typedef uint8_t (*update_pid_params_fun_t)(proto_pid_data_t *pid);
bool proto_register_update_pid_fun(update_pid_params_fun_t *update_pid_params_fun);


/*解析和获取一帧数据*/
uint16_t proto_deal_frame_data(protocol_package_t* protocol_package);
uint16_t proto_get_upload_frame(protocol_package_t* protocol_package);


#endif // _PROTO_V1_0_0_220621_H_