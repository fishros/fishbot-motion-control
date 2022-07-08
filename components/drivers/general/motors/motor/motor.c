
/**
 * @brief 文件描述：待更新
 * @author 小鱼 (fishros@foxmail.com)
 * @version V1.0.0
 * @date 2022-07-07
 * @copyright 版权所有：FishBot Open Source Organization
 */
#include "motor.h"

#define FISHBOT_MODLUE "MOTOR"

#define UPDATE_OUTPUT(motor_id, output)                                          \
    if (output > 0)                                                              \
    {                                                                            \
        gpio_set_level(motor_config_[motor_id].io_positive, 1);                  \
        gpio_set_level(motor_config_[motor_id].io_negative, 0);                  \
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, ledc_channel_map[motor_id], output); \
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, ledc_channel_map[motor_id]);      \
    }

/*configs*/
static uint8_t motor_num_ = 0;
static pid_ctrl_config_t *pid_config_ = 0;
static motor_config_t *motor_config_ = 0;
static uint8_t ledc_channel_map[] = {LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
/* PID\编码器 */
static pid_ctrl_block_handle_t pid_ctrl_block_handle_[MAX_MOTOR_NUM];
// static uint8_t *ledc_channel_[MAX_MOTOR_NUM];
static rotary_encoder_t *rotary_encoder_[MAX_MOTOR_NUM];

bool set_motor_config(uint8_t motor_num, motor_config_t *motor_configs, pid_ctrl_config_t *pid_configs)
{
    motor_num_ = motor_num;
    pid_config_ = pid_configs;
    motor_config_ = motor_configs;
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
    // get_config && init
    uint8_t i;
    gpio_config_t io_conf;
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
        .freq_hz = 5000,                      // frequency of PWM signal
        .speed_mode = LEDC_HIGH_SPEED_MODE,   // timer mode
        .timer_num = LEDC_TIMER_0,            // timer index
        .clk_cfg = LEDC_AUTO_CLK,             // Auto select the source clock
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    for (i = 0; i < motor_num_; i++)
    {
        // 编码器初始化
        rotary_encoder_[i] = create_rotary_encoder(i, motor_config_[i].io_encoder_positive, motor_config_[i].io_encoder_negative);

        // 方向控制IO初始化
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pin_bit_mask = (1ULL << motor_config_[i].io_positive) | (1ULL << motor_config_[i].io_negative);
        io_conf.pull_down_en = 0;
        io_conf.pull_up_en = 1;
        gpio_config(&io_conf);
        gpio_set_level(motor_config_[i].io_positive, 0);
        gpio_set_level(motor_config_[i].io_negative, 0);

        // PWM初始化
        ledc_channel_config_t ledc_channel = {
            .channel = ledc_channel_map[i],
            .duty = 0,
            .gpio_num = motor_config_[i].io_pwm,
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .hpoint = 0,
            .timer_sel = LEDC_TIMER_0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
        // pid 配置初始化
        pid_new_control_block(pid_config_ + i, pid_ctrl_block_handle_ + i);
    }

    return true;
}

static int32_t last_time_mm;
static int32_t current_time_mm;
static uint16_t delta_time_mm;
static int32_t tick_count[MAX_MOTOR_NUM];
static void motor_task(void *param)
{
    uint8_t i;
    // float spped_left, spped_right = 0;
    delta_time_mm = 0;
    last_time_mm = xTaskGetTickCount();
    current_time_mm = xTaskGetTickCount();
    while (1)
    {
        for (i = 0; i < motor_num_; i++)
        {
            delta_time_mm = last_time_mm - xTaskGetTickCount();
            // update ecoder
            tick_count[i] = rotary_encoder_[i]->get_counter_value(rotary_encoder_[i]);
            // ESP_LOGI(FISHBOT_MODLUE, "tick %d:%d", i, tick);
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
