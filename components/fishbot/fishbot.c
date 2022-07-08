/**
 * @brief 系统启动、初始化任务
 * @author 小鱼 (fishros@foxmail.com)
 * @version V1.0.0
 * @date 2022-07-03
 * @copyright 版权所有：FishBot Open Source Organization
 */
#include "fishbot.h"

#define FISHBOT_MODLUE "FISHBOT"

static fishbot_config_t fishbot_config = {
    .driver_version = "fbmcd.v1.0.0.220703",
    .hardware_version = "fbmch.v1.0.0.220721",
    .motors_num = 2,
};

bool fishbot_init(void)
{
    fishbot_configurate_init(&fishbot_config);
    ESP_LOGI(FISHBOT_MODLUE, "fishbot init with %s:%s", fishbot_config_get_driver_version(), fishbot_config_get_hardware_version());
    if (!fishbot_init_hardware())
        return false;

    return true;
}

bool fishbot_task_start(void)
{
    led_task_init();
    //motor_task_init(); //电机功能
    uart_protocol_init();
    return true;
}
bool fishbot_configurate_init(const fishbot_config_t *config)
{
    // TODO(小鱼) 从flash读取配置
    return true;
}

bool fishbot_init_hardware(void)
{
    led_init();
    if (!led_test())
        return false;
    if (!motor_init())
        return false;
    return true;
}

const fishbot_config_t *fishbot_get_configuration(void)
{
    return &fishbot_config;
}

const char *fishbot_config_get_driver_version(void)
{
    return fishbot_config.driver_version;
}

const char *fishbot_config_get_hardware_version()
{
    return fishbot_config.hardware_version;
}