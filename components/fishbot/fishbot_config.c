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
    .motors_num = 1,
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
            .kp = 50,
            .ki = 10 ,
            .kd = 0,
            .max_output = 8000,
            .min_output = -8000,
            .max_integral = 2000,
            .min_integral = -2000,
            .cal_type = PID_CAL_TYPE_POSITIONAL,
        },
    },
    {
        .init_param = {
            .kp = 100,
            .ki = 0,
            .kd = 0,
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
        .io_pwm = 4,
        .io_positive = 39,
        .io_negative = 40,
        .io_encoder_positive = 41,
        .io_encoder_negative = 42,
    },
    {
        .motor_id = 1,
        .io_pwm = 5,
        .io_positive = 44,
        .io_negative = 45,
        .io_encoder_positive = 13,
        .io_encoder_negative = 14,
    },
};

i2c_device_config_t i2c_device_config = {
    .scl_pin = 11,
    .sda_pin = 10,
    .i2c_num = I2C_NUM_0,
};

fishbot_wifi_config_t wifi_config = {
    .ssid = "fishbot",
    .pswd = "fishros.com",
    .ap_ssid = "fbmc",
    .ap_pswd = "",
    .mode = WIFI_MODE_STA,
    // .mode = WIFI_MODE_AP,
};

protocol_config_t protocol_config = {
    .mode = MODE_USB,
};

bool fishbot_config_init()
{
    // TODO(小鱼) 从flash读取配置
    set_led_config(DEFAULT_LED_NUM, led_configs);
    set_motor_config(DEFAULT_MOTOR_NUM, motor_configs, pid_config);
    set_i2c_device_config(&i2c_device_config);
    // usb 模式也开启wifi,通过手机查看日志信息和控制机器人
    set_wifi_config(&wifi_config);
    set_protocol_config(&protocol_config);
    /*注册pid更新回调函数*/
    proto_register_update_pid_fun(update_motor_pid_param);
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
