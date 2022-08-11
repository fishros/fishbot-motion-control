/**
 * @brief 启动入口文件
 * @author 小鱼 (fishros@foxmail.com)
 * @version V1.0.0
 * @date 2022-06-21
 * @copyright 版权所有：FishBot Open Source Organization
 */

#include "fishbot.h"

/**
 * @brief 主入口函数
 *
 */
void app_main(void)
{
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(ret);

    if (fishbot_init() == false)
    {
        while (1)
            ;
    }
    fishbot_task_init();
}
