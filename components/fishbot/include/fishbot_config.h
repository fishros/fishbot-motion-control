/**
 * @brief 文件描述：待更新
 * @author 小鱼 (fishros@foxmail.com)
 * @version V1.0.0
 * @date 2022-07-07
 * @copyright 版权所有：FishBot Open Source Organization
 */
#ifndef _FISHBOT_CONFIG_H_
#define _FISHBOT_CONFIG_H_

#include "led.h"
#include "motor.h"
#include "pid_controller.h"
#include "rotary_encoder.h"
#include "oled.h"
#include "mpu6050.h"
#include "wifi.h"
#include "protocol.h"
#include "store.h"
#include "key.h"

#define FISHBOT_SOFTWARE_VERSION_STRING_MAX_LEN (32 + 1)
#define FISHBOT_HARDWARE_VERSION_STRING_MAX_LEN (32 + 1)

// LED的数量
#define DEFAULT_LED_NUM 1
// 蓝色灯的ID
#define LED_BLUE 0
// 默认的电机数量
#define DEFAULT_MOTOR_NUM 2

#define WIFI_MODE_COONFIG_NAME "wifi_config"
#define PROTO_COONFIG_NAME "proto_config"

/**
 * @brief 系统配置结构体
 *
 */
typedef struct
{
    char driver_version[FISHBOT_SOFTWARE_VERSION_STRING_MAX_LEN];
    char hardware_version[FISHBOT_HARDWARE_VERSION_STRING_MAX_LEN];
    uint8_t motors_num;
} fishbot_config_t;

/**
 * @brief 初始化配置文件，从nvs中加载配置文件
 *
 * @return true
 * @return false
 */
bool fishbot_config_init();

/**
 * @brief 首次上电初始化配置
 *
 * @return true
 * @return false
 */
bool fishbot_first_startup_config_init();

/**
 * @brief 对外提供接口，获取配置文件
 *
 * @return const fishbot_config_t*
 */
const fishbot_config_t *fishbot_get_config(void);

/**
 * @brief 获取系统驱动版本信息
 *
 * @return const char* fbmch:fishbot motion control driver
 */
const char *fishbot_config_get_driver_version();

/**
 * @brief 获取系统硬件版本信息
 *
 * @return const char* fbmch:fishbot motion control hardware
 */
const char *fishbot_config_get_hardware_version();

// update motor pid config
// update motor scale config
// update wifi config
// update udp server config
// update transform config

#endif // _FISHBOT_CONFIG_H_
