#include "key.h"

#define FISHBOT_MODULE "KEY"

static xQueueHandle gpio_evt_queue = NULL;

static fishbot_proto_config_t protocol_config = {
    .mode = PROTO_MODE_UPDATE,
};

static void gpio_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void key_detect_task(void *arg)
{
    uint32_t io_num;
    time_t now, last_time;
    time(&now);
    time(&last_time);
    static uint8_t count = 0;
    for (;;)
    {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY))
        {
            ESP_LOGI(FISHBOT_MODULE, "GPIO intr \n");
            if (gpio_get_level(io_num) == 1)
                count++;
            time(&now);
            if (now - last_time > 1.0)
            {
                last_time = now;
                count = 0;
            }

            if (count > 2)
            {
                // 切换模式到AP(协议自动切换到UDP_SERVER)
                proto_update_potocol_mode_config(&protocol_config);
                // 自动重启一下
                esp_restart();
            }
        }
    }
}

void key_init(void)
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    // bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = (1ULL << GPIO_INPUT_IO);
    // set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
    // change gpio intrrupt type for one pin
    gpio_set_intr_type(GPIO_INPUT_IO, GPIO_INTR_POSEDGE);
    // create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(5, sizeof(uint32_t));
    // install gpio isr service
    gpio_install_isr_service(0);
    // hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO, gpio_isr_handler, (void *)GPIO_INPUT_IO);
}

void key_task_init(void)
{
    xTaskCreate(key_detect_task, "key_detect_task", 2048, NULL, 4, NULL);
}