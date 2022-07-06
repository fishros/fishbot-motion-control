
#include "motor.h"

#define FISHBOT_MODLUE "MOTOR"

#define UPDATE_OUTPUT(motor_id, output)                                          \
    if (output > 0)                                                              \
    {                                                                            \
        gpio_set_level(motor_config[motor_id].io_positive, 1);                   \
        gpio_set_level(motor_config[motor_id].io_negative, 0);                   \
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, ledc_channel_map[motor_id], output); \
    }

static motor_config_t motor_config[] = {
    {.motor_id = 0,
     .io_pwm = 18,
     .io_positive = 22,
     .io_negative = 23,
     .io_encoder_positive = 32,
     .io_encoder_negative = 33},
    {.motor_id = 1,
     .io_pwm = 19,
     .io_positive = 13,
     .io_negative = 12,
     .io_encoder_positive = 25,
     .io_encoder_negative = 26},
};

static rotary_encoder_t *rotary_encoder[MOTOR_NUM];
static uint8_t ledc_channel_map[MOTOR_NUM] = {LEDC_CHANNEL_0, LEDC_CHANNEL_1};

bool motor_config_init()
{
    // TODO 小鱼(read pid\io_config from nvs)
    return true;
}

/**
 * @brief 初始化电机，编码器&PWM
 *
 * @return true
 * @return false
 */
bool motor_init(void)
{
    uint8_t i;
    gpio_config_t io_conf;
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
        .freq_hz = 5000,                      // frequency of PWM signal
        .speed_mode = LEDC_HIGH_SPEED_MODE,   // timer mode
        .timer_num = LEDC_TIMER_0,            // timer index
        .clk_cfg = LEDC_AUTO_CLK,             // Auto select the source clock
    };
    ledc_timer_config(&ledc_timer);

    for (i = 0; i < MOTOR_NUM; i++)
    {
        // 编码器初始化
        rotary_encoder[motor_config[i].motor_id] = create_rotary_encoder(i, motor_config[i].io_encoder_positive, motor_config[i].io_encoder_negative);
        // 方向控制IO初始化
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pin_bit_mask = (1ULL << motor_config[i].io_positive) | (1ULL << motor_config[i].io_negative);
        io_conf.pull_down_en = 0;
        io_conf.pull_up_en = 1;
        gpio_config(&io_conf);
        gpio_set_level(motor_config[i].io_positive, 0);
        gpio_set_level(motor_config[i].io_negative, 0);
        // PWM初始化
        ledc_channel_config_t ledc_channel = {
            .channel = ledc_channel_map[motor_config[i].motor_id],
            .duty = 0,
            .gpio_num = motor_config[i].io_pwm,
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .hpoint = 0,
            .timer_sel = LEDC_TIMER_0,
        };
        ledc_channel_config(&ledc_channel);
        // pid 配置初始化
    }

    return true;
}

static void motor_task(void *param)
{
    uint8_t i;
    // float spped_left, spped_right = 0;
    // int32_t last_time_mm = xTaskGetTickCount();

    while (1)
    {
        for (i = 0; i < MOTOR_NUM; i++)
        {
            // update ecoder
            int32_t tick = rotary_encoder[motor_config[i].motor_id]->get_counter_value(rotary_encoder[motor_config[i].motor_id]);
            ESP_LOGI(FISHBOT_MODLUE, "tick %d:%d", motor_config[i].motor_id, tick);
            // calcute speed
            UPDATE_OUTPUT(i, 2000);
            // update pid
            // update output
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }
    vTaskDelete(NULL);
}

/**
 * @brief 电机控制任务
 *
 * @param param
 */
void motor_task_init()
{
    xTaskCreate(motor_task, "motor_task", 5 * 1024, NULL, 5, NULL);
}
