#ifndef _UDP_CLIENT_H_
#define _UDP_CLIENT_H_
#include <lwip/netdb.h>
#include <string.h>
#include <sys/param.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "proto_utils.h"


typedef struct
{
    char server_address[16];
    uint16_t server_port;
} mode_wifi_udp_pc_config_t;


/**
 * @brief 设置UDP客户端通信相关配置
 *
 * @param udp_pc_config_t
 * @return true
 * @return false
 */
bool set_udp_client_config(mode_wifi_udp_pc_config_t *udp_pc_config_t);

/**
 * @brief UDP客户端通信初始化，主要初始化收发队列（移到配置函数？）
 *
 * @param rx_queue
 * @param tx_queue
 * @return true
 * @return false
 */
bool udp_client_protocol_init(xQueueHandle *rx_queue, xQueueHandle *tx_queue);

/**
 * @brief UDP客户端收发任务初始化
 *
 * @return true
 * @return false
 */
bool udp_client_protocol_task_init(void);

/**
 * @brief 根据配置连接服务端
 *
 * @return true
 * @return false
 */
bool udp_client_connect(void);

// bool update_udp_server_config(); V1.0.0暂不添加动态UDP参数&WIFI功能

#endif  // _UDP_CLIENT_H_
