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

#include "fishbot_config.h"
#include "nvs_flash.h"

#include "esp_log.h"
#include "led.h"
#include "motor.h"
// #include "protocol/include/uart_protocol.h"
#define FISHBOT_SOFTWARE_VERSION_STRING_MAX_LEN (32 + 1)
#define FISHBOT_HARDWARE_VERSION_STRING_MAX_LEN (32 + 1)

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
bool fishbot_task_init(void);

#endif // _FISHBOT_H_
