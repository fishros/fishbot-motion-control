/**
 * @brief 文件描述：待更新
 * @author 小鱼 (fishros@foxmail.com)
 * @version V1.0.0
 * @date 2022-07-07
 * @copyright 版权所有：FishBot Open Source Organization
 */
#include "fishbot_config.h"

static fishbot_config_t fishbot_config = {
    .driver_version = "fbmcd.v1.0.0.220703",
    .hardware_version = "fbmch.v1.0.0.220721",
    .motors_num = 2,
};

static led_config_t led_configs[] = {
    {
        .id = 0,
        .led_pin = 2,
        .led_polarity = LED_POL_POS,
    },
};

static pid_ctrl_config_t pid_config[] = {
    {
        .init_param = {
            .kp = 300,
            .ki = 0,
            .kd = 200,
            .max_output = 8000,
            .min_output = -8000,
            .max_integral = 2000,
            .min_integral = -2000,
            .cal_type = PID_CAL_TYPE_POSITIONAL,
        },
    },
    {
        .init_param = {
            .kp = 300,
            .ki = 0,
            .kd = 200,
            .max_output = 8000,
            .min_output = -8000,
            .max_integral = 2000,
            .min_integral = -2000,
            .cal_type = PID_CAL_TYPE_POSITIONAL,
        },
    },
};

static motor_config_t motor_configs[] = {
    {
        .motor_id = 0,
        .io_pwm = 18,
        .io_positive = 22,
        .io_negative = 23,
        .io_encoder_positive = 32,
        .io_encoder_negative = 33,
    },
    {
        .motor_id = 1,
        .io_pwm = 19,
        .io_positive = 13,
        .io_negative = 12,
        .io_encoder_positive = 25,
        .io_encoder_negative = 26,
    },
};

bool fishbot_config_init()
{
    // TODO(小鱼) 从flash读取配置
    set_led_config(DEFAULT_LED_NUM,led_configs);
    // bool set_motor_config(uint8_t motor_num_, motor_config_t *motor_configs, pid_ctrl_config_t *pid_configs);
    set_motor_config(DEFAULT_MOTOR_NUM, motor_configs, pid_config);
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
