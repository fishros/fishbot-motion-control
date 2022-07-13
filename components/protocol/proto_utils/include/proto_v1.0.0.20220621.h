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

/**
 * @brief 数据ID定义
 *
 */
typedef enum
{
    DATA_ENCODER = 0x01,  // 0x01-左右轮编码器
    DATA_IMU = 0x02,      // 0x02-IMU传感器
    DATA_SPEED = 0x03,    // 0x03-速度控制数据
    DATA_PID = 0x04,      // 0x04-PID设置
    DATA_VER_INFO = 0x05, // 0x05-版本信息
} proto_data_id_t;

/**
 * @brief 数据传输方向定义
 *
 */
typedef enum
{
    DATA_TO_MASTER = 0x01, // 0X01,反馈数据（底盘向主控）
    DATA_TO_FBMC = 0x02,   // 0X02,命令数据（主控向底盘）
} proto_data_direction_t;

/**
 * @brief PID数据结构体
 *
 */
typedef struct
{
    float kp;
    float ki;
    float kd;
} proto_pid_data_t;

/**
 * @brief 电机编码器数据结构体
 *
 */
typedef struct
{
    uint32_t motor_encoder[MAX_MOTOR_NUM]; // 电机的编码器数据
} proto_motor_encoder_data_t;

/**
 * @brief 电机速度控制结构体
 *
 */
typedef struct
{
    uint16_t motor_speed[MAX_MOTOR_NUM]; // 单位mm/s
} proto_motor_speed_ctrl_data_t;

/**
 * @brief IMU数据结构体
 *
 */
typedef struct
{
    float accel[3]; // 加速度
    float gyro[3];  // 重力加速度
    float quat[4];  // 四元数 xyzw
} proto_imu_data_t;

/**
 * @brief 定义更新PID数据的钩子
 *
 */
typedef uint8_t (*update_pid_params_fun_t)(proto_pid_data_t *pid);

/**
 * @brief 注册更新PID数据的钩子函数
 *
 * @param update_pid_params_fun
 * @return true
 * @return false
 */
bool proto_register_update_pid_fun(update_pid_params_fun_t *update_pid_params_fun);

/**
 * @brief 定义更新速度数据的钩子
 *
 */
typedef uint8_t (*update_speed_params_fun_t)(proto_motor_speed_ctrl_data_t *pid);

/**
 * @brief 注册更新电机速度数据的钩子
 *
 * @param update_motor_speed_ctrl_fun
 * @return true
 * @return false
 */
bool proto_register_update_motor_speed_fun(update_speed_params_fun_t *update_motor_speed_ctrl_fun);

/**
 * @brief 解析和获取一帧数据（根据数据调用不同的钩子完成数据的更新到各个模块）
 *
 * @param protocol_package 数据包指针
 * @return uint16_t
 */
uint16_t proto_deal_frame_data(protocol_package_t *protocol_package);

/**
 * @brief 获取一个需要上传的数据帧（包含传感器的信息组合）
 *
 * @param **protocol_package  指向数据包指针的指针
 * @return uint16_t
 */
uint16_t proto_get_upload_frame(protocol_package_t **protocol_package);

#endif // _PROTO_V1_0_0_220621_H_