/**
 * @brief 文件描述：待更新
 * @author 小鱼 (fishros@foxmail.com)
 * @version V1.0.0
 * @date 2022-07-07
 * @copyright 版权所有：FishBot Open Source Organization
 */
#include "motor.h"

#define FISHBOT_MODLUE "MOTOR"

#ifdef DRIVER_USE_TB6612
#define UPDATE_OUTPUT(motor_id, output)                                     \
    if (output > 0)                                                         \
    {                                                                       \
        gpio_set_level(motor_config_[motor_id].io_positive, 1);             \
        gpio_set_level(motor_config_[motor_id].io_negative, 0);             \
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, ledc_channel_map[motor_id],     \
                      (int)output);                                         \
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, ledc_channel_map[motor_id]); \
    }                                                                       \
    else                                                                    \
    {                                                                       \
        gpio_set_level(motor_config_[motor_id].io_positive, 0);             \
        gpio_set_level(motor_config_[motor_id].io_negative, 1);             \
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, ledc_channel_map[motor_id],     \
                      -1 * (int)output);                                    \
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, ledc_channel_map[motor_id]); \
    }

#endif

#ifdef DRIVER_USE_DRV8833

#define UPDATE_OUTPUT(motor_id, output)                                                    \
    if (output > 0)                                                                        \
    {                                                                                      \
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, ledc_channel_map[2 * motor_id], (int)output);  \
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, ledc_channel_map[2 * motor_id]);            \
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, ledc_channel_map[2 * motor_id+1], 0);           \
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, ledc_channel_map[2 * motor_id+1]);            \
    }                                                                                      \
    else                                                                                   \
    {                                                                                      \
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, ledc_channel_map[2 * motor_id], 0);           \
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, ledc_channel_map[2 * motor_id]);            \
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, ledc_channel_map[2*motor_id+1], -1 * (int)output); \
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, ledc_channel_map[2*motor_id+1]);                \
    }

#endif

/*configs*/
static uint8_t motor_num_ = 0;
static pid_ctrl_config_t *pid_config_ = 0;
static motor_config_t *motor_config_ = 0;
static uint8_t ledc_channel_map[] = {LEDC_CHANNEL_0, LEDC_CHANNEL_1,
                                     LEDC_CHANNEL_2, LEDC_CHANNEL_3};

static pid_ctrl_block_handle_t
    pid_ctrl_block_handle_[MAX_MOTOR_NUM];               // PID控制结构体
static rotary_encoder_t *rotary_encoder_[MAX_MOTOR_NUM]; // 编码器配置
static int16_t target_speeds[MAX_MOTOR_NUM] = {0, 0};    // 电机当前速度，单位mm/s
static uint16_t tick_to_mms[MAX_MOTOR_NUM] = {
    62.011394, 62.011394}; // 电机的编码器和距离换算出的值
static proto_data_motor_encoder_t
    proto_motor_encoder_data_; // 上传存储的编码器数据

bool set_motor_config(uint8_t motor_num, motor_config_t *motor_configs,
                      pid_ctrl_config_t *pid_configs)
{
    motor_num_ = motor_num;
    pid_config_ = pid_configs;
    motor_config_ = motor_configs;
    return true;
}

uint8_t update_motor_pid_param(fishbot_pid_config_t *proto_pid_data)
{
    // pid_update_parameters(pid_ctrl_block_handle_[0],);
    return true;
}

uint8_t update_motor_spped_fun(
    proto_motor_speed_ctrl_data_t *motor_spped_ctrl)
{
    static uint8_t index;
    for (index = 0; index < motor_num_; index++)
    {
        target_speeds[index] = motor_spped_ctrl->motor_speed[index];
    }
    return index;
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
        rotary_encoder_[i] =
            create_rotary_encoder(i, motor_config_[i].io_encoder_positive,
                                  motor_config_[i].io_encoder_negative);
        // pid 配置初始化
        pid_new_control_block(pid_config_ + i, pid_ctrl_block_handle_ + i);
    }

#ifdef DRIVER_USE_TB6612
    for (i = 0; i < motor_num_; i++)
    {
        // 方向控制IO初始化
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pin_bit_mask = (1ULL << motor_config_[i].io_positive) |
                               (1ULL << motor_config_[i].io_negative);
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
    }
#endif

#ifdef DRIVER_USE_DRV8833
    for (i = 0; i < motor_num_; i++)
    {
        // PWM通道1初始化
        ledc_channel_config_t ledc_channel1 = {
            .channel = ledc_channel_map[2 * i],
            .duty = 0,
            .gpio_num = motor_config_[i].io_positive,
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .hpoint = 0,
            .timer_sel = LEDC_TIMER_0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel1));
        // PWM通道2初始化
        ledc_channel_config_t ledc_channel2 = {
            .channel = ledc_channel_map[2 * i + 1],
            .duty = 0,
            .gpio_num = motor_config_[i].io_negative,
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .hpoint = 0,
            .timer_sel = LEDC_TIMER_0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel2));
    }
#endif

    proto_set_motor_encoder_data(&proto_motor_encoder_data_);
    proto_register_update_motor_speed_fun(update_motor_spped_fun);
    ESP_LOGI(FISHBOT_MODLUE, "init success!");
    return true;
}

static void motor_task(void *param)
{
    static uint8_t i = 0;
    static uint16_t last_time_mm = 0;              //上一次的时间
    static uint16_t current_time_mm = 0;           //当前时间
    static uint16_t delta_time_mm = 0;             // 间隔时间
    static int32_t tick_count[MAX_MOTOR_NUM];      // 编码器tick数
    static int32_t last_tick_count[MAX_MOTOR_NUM]; // 上一轮编码器tick数
    static int32_t current_speeds[MAX_MOTOR_NUM];  // 电机当前速度，单位mm/s
    static float output_pwm_[MAX_MOTOR_NUM];       //输出的PWM值

    last_time_mm = xTaskGetTickCount();
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
        current_time_mm = xTaskGetTickCount();
        delta_time_mm = current_time_mm - last_time_mm;
        last_time_mm = current_time_mm;
        for (i = 0; i < motor_num_; i++)
        {
            // update current data ecoder
            tick_count[i] = rotary_encoder_[i]->get_counter_value(rotary_encoder_[i]);
            // 当前轮子转一圈产生3293个脉冲（tick），轮子直径65mm
            // 65*3.1415926mm/3293tick= 0.062011394
            current_speeds[i] =
                (tick_count[i] - last_tick_count[i]) * tick_to_mms[i] / delta_time_mm;
            // update pid
            pid_compute(pid_ctrl_block_handle_[i],
                        target_speeds[i] - current_speeds[i], output_pwm_ + i);
            UPDATE_OUTPUT(i, output_pwm_[i]);
            proto_motor_encoder_data_.motor_encoder[i] = tick_count[i];
            // update last data
            last_tick_count[i] = tick_count[i];
        }
        proto_set_motor_encoder_data(&proto_motor_encoder_data_);
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
