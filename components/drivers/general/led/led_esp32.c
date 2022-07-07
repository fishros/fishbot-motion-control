/**
 * @brief LED驱动，状态指示灯
 * @author 小鱼 (fishros@foxmail.com)
 * @version V1.0.0
 * @date 2022-07-03
 * @copyright 版权所有：FishBot Open Source Organization
 */

/* fishbot compont includes */
#include "led.h"

#define FISHBOT_MODLUE "LED"

static bool is_init = false;
static uint8_t led_num_ = 0;
static led_config_t *led_config_;

void set_led_config(uint8_t led_nums, led_config_t *led_configs)
{
    led_config_ = led_configs;
    led_num_ = led_nums;
}

void led_init(void)
{
    if (is_init)
        return;
    uint8_t i;
    for (i = 0; i < led_num_; i++)
    {
        gpio_config_t io_conf;
        // disable interrupt
        io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
        // bit mask of the pins that you want to set,e.g.GPIO18/19
        io_conf.pin_bit_mask = (1ULL << 2);
        // disable pull-down mode
        io_conf.pull_down_en = 0;
        // disable pull-up mode
        io_conf.pull_up_en = 1;
        // set as output mode
        io_conf.mode = GPIO_MODE_OUTPUT;
        // configure GPIO with the given settings
        gpio_config(&io_conf);
        led_set(i, 0);
    }

    is_init = true;
}
bool led_test(void)
{
    led_set(LED_BLUE, 1);
    vTaskDelay(1000 / portTICK_RATE_MS);
    led_set(LED_BLUE, 0);
    vTaskDelay(1000 / portTICK_RATE_MS);
    led_set(LED_BLUE, 1);
    return is_init;
}

void led_set(uint8_t id, bool value)
{
    if (id > led_num_ || id == led_num_)
    {
        return;
    }

    if (led_config_[id].led_polarity == LED_POL_NEG)
    {
        value = !value;
    }

    if (value)
    {
        gpio_set_level(led_config_[id].led_pin, 1);
    }
    else
    {
        gpio_set_level(led_config_[id].led_pin, 0);
    }
}

static void led_task(void *param)
{
    while (1)
    {
        led_test();
        ESP_LOGI(FISHBOT_MODLUE, "led task runing..");
    }
}

void led_task_init(void)
{
    // led_task
    xTaskCreate(led_task, "led_task", 2 * 1024, NULL, 5, NULL);
}
