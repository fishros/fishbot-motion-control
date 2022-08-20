/**
 * @brief 文件描述：待更新
 * @author 小鱼 (fishros@foxmail.com)
 * @version V1.0.0
 * @date 2022-07-07
 * @copyright 版权所有：FishBot Open Source Organization
 */
#include "fishbot_config.h"

#define FISHBOT_MODLUE "FISHBOT_COFIG"

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

static motor_config_t motor_configs[] = {
    {
        .motor_id = 0,
        .io_pwm = 5,
        .io_positive = 12,
        .io_negative = 13,
        .io_encoder_positive = 26,
        .io_encoder_negative = 25,
    },
    {
        .motor_id = 1,
        .io_pwm = 4,
        .io_positive = 22,
        .io_negative = 23,
        .io_encoder_positive = 32,
        .io_encoder_negative = 33,
    },
};

i2c_device_config_t i2c_device_config = {
    .scl_pin = 19,
    .sda_pin = 18,
    .i2c_num = I2C_NUM_0,
};

static pid_ctrl_config_t pid_config[] = {
    {
        .init_param =
            {
                .kp = 50,
                .ki = 10,
                .kd = 0,
                .max_output = 8000,
                .min_output = -8000,
                .max_integral = 2000,
                .min_integral = -2000,
                .cal_type = PID_CAL_TYPE_POSITIONAL,
            },
    },
    {
        .init_param =
            {
                .kp = 50,
                .ki = 10,
                .kd = 0,
                .max_output = 8000,
                .min_output = -8000,
                .max_integral = 2000,
                .min_integral = -2000,
                .cal_type = PID_CAL_TYPE_POSITIONAL,
            },
    },
};

fishbot_wifi_config_t wifi_config = {
    .ssid = "fishbot", .password = "fishros.com", .mode = WIFI_MODE_STA,
    // .mode = WIFI_MODE_AP,
};

fishbot_proto_config_t protocol_config = {
    .mode = PROTO_MODE_WIFI_UDP_SERVER,
    .ip = "192.168.0.103",
    .port = 3474,
    .bautrate = 115200,
};

bool fishbot_first_startup_config_init()
{
    int8_t configured;
    nvs_read_uint8("configured", &configured);
    ESP_LOGI(FISHBOT_MODLUE, "config=%d", configured);
    if (configured == NVS_DATA_UINT8_NONE)
    {
        nvs_write_struct(WIFI_MODE_COONFIG_NAME, &wifi_config,
                         sizeof(fishbot_wifi_config_t));
        nvs_write_struct(PROTO_COONFIG_NAME, &protocol_config,
                         sizeof(fishbot_proto_config_t));
        nvs_write_uint8("configured", 1);
    }
    else
    {
        nvs_read_struct(WIFI_MODE_COONFIG_NAME, &wifi_config,
                        sizeof(fishbot_wifi_config_t));
        nvs_read_struct(PROTO_COONFIG_NAME, &protocol_config,
                        sizeof(fishbot_proto_config_t));
    }
    return true;
}

uint8_t fishbot_update_motor_pid_param(fishbot_pid_config_t *proto_pid_data)
{
    // @TODO update nvs. fish(fishros@foxmail.com)
    update_motor_pid_param(proto_pid_data);
    return true;
}

uint8_t fishbot_update_wifi_config(fishbot_wifi_config_t *fishbot_wifi_config)
{
    // 将更新后的配置写入数据库
    nvs_write_struct(WIFI_MODE_COONFIG_NAME, fishbot_wifi_config,
                     sizeof(fishbot_wifi_config_t));
    return true;
}

uint8_t fishbot_update_proto_config(
    fishbot_proto_config_t *fishbot_proto_config)
{
    // 将更新后的配置写入数据库
    ESP_LOGI(FISHBOT_MODLUE, "write1 proto mode=%d port=%d baudrate=%d ip=%s",
             fishbot_proto_config->mode, fishbot_proto_config->port,
             fishbot_proto_config->bautrate, fishbot_proto_config->ip);
    if (fishbot_proto_config->mode == PROTO_MODE_UPDATE)
    {
        // 切换模式
        if (protocol_config.mode == PROTO_MODE_UART)
        {
            protocol_config.mode = PROTO_MODE_WIFI_UDP_CLIENT;
        }
        else if (protocol_config.mode == PROTO_MODE_WIFI_UDP_CLIENT)
        {
            protocol_config.mode = PROTO_MODE_WIFI_UDP_SERVER;
        }
        else if (protocol_config.mode == PROTO_MODE_WIFI_UDP_SERVER)
        {
            protocol_config.mode = PROTO_MODE_UART;
        }
    }

    if (fishbot_proto_config->port != 0)
    {
        protocol_config.port = fishbot_proto_config->port;
    }

    if (fishbot_proto_config->bautrate != 0)
    {
        protocol_config.bautrate = fishbot_proto_config->bautrate;
    }

    if (fishbot_proto_config->ip[0] != 0)
    {
        sprintf(protocol_config.ip, "%s", fishbot_proto_config->ip);
    }

    ESP_LOGI(FISHBOT_MODLUE, "write2 proto mode=%d port=%d baudrate=%d ip=%s",
             protocol_config.mode, protocol_config.port,
             protocol_config.bautrate, protocol_config.ip);

    nvs_write_struct(PROTO_COONFIG_NAME, &protocol_config,
                     sizeof(fishbot_proto_config_t));
    return true;
}

bool fishbot_config_init()
{
    /*读取配置，若是初次上电则初始化数据库*/
    fishbot_first_startup_config_init();
    ESP_LOGI(FISHBOT_MODLUE, "proto mode=%d port=%d baudrate=%d ip=%s",
             protocol_config.mode, protocol_config.port, protocol_config.bautrate,
             protocol_config.ip);

    set_wifi_config(&wifi_config);
    set_protocol_config(&protocol_config);

    set_led_config(DEFAULT_LED_NUM, led_configs);
    set_motor_config(DEFAULT_MOTOR_NUM, motor_configs, pid_config);
    set_i2c_device_config(&i2c_device_config);

    /*注册数据更新回调函数*/
    proto_register_update_pid_fun(fishbot_update_motor_pid_param);
    proto_register_update_wifi_config_fun(fishbot_update_wifi_config);
    proto_register_update_potocol_config_fun(fishbot_update_proto_config);

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
