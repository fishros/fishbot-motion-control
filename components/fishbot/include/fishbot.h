/**
 * @brief 系统启动、初始化任务
 * @author 小鱼 (fishros@foxmail.com)
 * @version V1.0.0
 * @date 2022-07-03
 * @copyright 版权所有：FishBot Open Source Organization
 */
#ifndef _FISHBOT_H_
#define _FISHBOT_H_
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"

#include "esp_log.h"
#include "led.h"
#include "motor.h"

#define FISHBOT_SOFTWARE_VERSION_STRING_MAX_LEN (32 + 1)
#define FISHBOT_HARDWARE_VERSION_STRING_MAX_LEN (32 + 1)

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
 * @brief 加载配置，初始化硬件
 *
 * @return true
 * @return false
 */
bool fishbot_init(void);

/**
 * @brief 初始化硬件，系统检查
 *
 * @return true
 * @return false
 */
bool fishbot_init_hardware(void);

/**
 * @brief 各项任务初始化，具体见：TODO(小鱼) 增加系统说明文档链接
 *
 * @return true
 * @return false
 */
bool fishbot_task_start(void);

/**
 * @brief 初始化配置文件，从nvs中加载配置文件
 *
 * @param configs
 * @return true
 * @return false
 */
bool fishbot_configurate_init(const fishbot_config_t *configs);

/**
 * @brief 对外提供接口，获取配置文件
 *
 * @return const fishbot_config_t*
 */
const fishbot_config_t *fishbot_get_configuration(void);

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

#endif // _FISHBOT_H_
