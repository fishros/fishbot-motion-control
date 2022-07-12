/**
 * @brief 文件描述：待更新
 * @author 小鱼 (fishros@foxmail.com)
 * @version V1.0.0
 * @date 2022-07-07
 * @copyright 版权所有：FishBot Open Source Organization
 */
#ifndef __MWIFI_H__
#define __MWIFI_H__

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
#define WIF_MAX_AP_CONNECTION 2

typedef struct
{
    char ssid[32];
    char pswd[64];
    char ap_ssid[32];
    char ap_pswd[64];
    wifi_mode_t mode;
} fishbot_wifi_config_t;

typedef enum
{
    WIFI_STATUS_STA_DISCONECTED = 0,
    WIFI_STATUS_STA_CONNECTED = 1,
    WIFI_STATUS_AP_READY,
} wifi_status_t;

bool set_wifi_config(fishbot_wifi_config_t *wifi_config);

/**
 * @brief 连接到一个热点
 *
 * @param ssid
 * @param pswd
 * @return true
 * @return false
 */
bool wifi_set_as_sta(char *ssid, char *pswd);

/**
 * @brief 发射一个热点
 *
 * @param ssid
 * @param pswd
 * @return true
 * @return false
 */
bool wifi_set_as_ap(char *ssid, char *pswd);

/**
 * @brief wifi初始化
 *
 * @return true
 * @return false
 */
bool wifi_init(void);

/**
 * @brief Get the wifi ip address
 *
 * @param *ip_address
 * @return wifi_status_t
 */
wifi_status_t get_wifi_ip(char *ip_address);

#endif