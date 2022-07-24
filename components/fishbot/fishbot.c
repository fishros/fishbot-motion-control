/**
 * @brief 系统启动、初始化任务
 * @author 小鱼 (fishros@foxmail.com)
 * @version V1.0.0
 * @date 2022-07-03
 * @copyright 版权所有：FishBot Open Source Organization
 */
#include "fishbot.h"

#define FISHBOT_MODLUE "FISHBOT"

#define FISHBOT_ERROR_CHECK(MODULE, RESULT) \
    if (RESULT)                             \
        ESP_LOGI(MODULE, "init success~!"); \
    else                                    \
        ESP_LOGE(MODULE, "init failed~!");

bool fishbot_init(void)
{
    fishbot_config_init();
    ESP_LOGI(FISHBOT_MODLUE, "fishbot init with %s:%s",
             fishbot_config_get_driver_version(),
             fishbot_config_get_hardware_version());
    if (!fishbot_init_hardware())
        return false;

    return true;
}

bool fishbot_task_init(void)
{
    led_task_init();
    motor_task_init();
    mpu6050_task();
    protocol_task_init();
    esp_log_level_set("MPU6050",ESP_LOG_NONE); // 不输出IMU模块日志
    return true;
}

bool fishbot_init_hardware(void)
{
    if (!led_init())
        return false;
    if (!motor_init())
        return false;
    if (!i2c_device_init())
        return false;
    if (!oled_init())
        return false;
    if (!mpu6050_task_init())
        return false;
    if (!wifi_init())
        return false;
    if (!protocol_init())
        return false;
    char host[16];
    if (get_wifi_ip(host) != WIFI_STATUS_STA_DISCONECTED)
    {
        oled_show_ascii_auto_line(" WIFI SUCCESS ");
        oled_show_ascii_auto_line(host);
    }
    return true;
}
