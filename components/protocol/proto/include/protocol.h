#ifndef _PROTOCOL_H_
#define _PROTOCOL_H_
#include "freertos/FreeRTOS.h"

#define PROTO_V1_0_0_220621

#include "proto_utils.h"
#ifdef PROTO_V1_0_0_220621
#include "proto_v1.0.0.20220621.h"
#endif

#include "udp_client_protocol.h"
#include "uart_protocol.h"

/**
 * @brief 定义FISHBOT的通讯模式，目前提供USB串口通信，UDP手机APP通信和UDP电脑端通信。其中UDP_PC需要提供，SSID&PSWD HOST&PORT
 *
 */
typedef enum
{
    MODE_USB = 0,
    MODE_WIFI_UDP_APP,
    MODE_WIFI_UDP_PC,
} fishbot_mode_t;

typedef struct
{
    /* data */
} mode_usb_config_t;

typedef struct
{
    /* data */
} mode_wifi_udp_app_config_t;

typedef struct
{
    char server_address[16];
    u_int16_t server_port;
} mode_wifi_udp_pc_config_t;

typedef struct
{
    fishbot_mode_t mode;
    mode_wifi_udp_pc_config_t wifi_udp_pc_config;
    mode_usb_config_t udp_config;
} protocol_config_t;


/**
 * @brief 设置协议模块配置
 * 
 * @param protocol_config* 配置指针
 * @return true 
 * @return false 
 */
bool set_protocol_config(protocol_config_t *protocol_config);

/**
 * @brief 协议模块初始化
 * 
 * @return true 
 * @return false 
 */
bool protocol_init(void);

/**
 * @brief 协议任务初始化
 * 
 * @return true 
 * @return false 
 */
bool protocol_task_init(void);

#endif // _PROTOCOL_H_
