/**
 * @brief 系统启动、初始化任务
 * @author 小鱼 (fishros@foxmail.com)
 * @version V1.0.0
 * @date 2022-07-03
 * @copyright 版权所有：FishBot Open Source Organization
 */
#include "fishbot.h"

#define FISHBOT_MODLUE "FISHBOT"

bool fishbot_init(void)
{
    fishbot_config_init();
    ESP_LOGI(FISHBOT_MODLUE, "fishbot init with %s:%s", fishbot_config_get_driver_version(), fishbot_config_get_hardware_version());
    if (!fishbot_init_hardware())
        return false;

    return true;
}

bool fishbot_task_init(void)
{
    led_task_init();
    motor_task_init();
    return true;
}

bool fishbot_init_hardware(void)
{
    led_init();
    if (!led_test())
        return false;
    if (!motor_init())
        return false;
    if (!oled_init())
        return false;
    return true;
}
