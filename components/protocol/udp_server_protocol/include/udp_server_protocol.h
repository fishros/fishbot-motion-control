#ifndef _UDP_SERVER_PROTOCOL_H_
#define _UDP_SERVER_PROTOCOL_H_
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
#include "proto_utils.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>


/**
 * @brief 设置UDP服务端通信相关配置
 *
 * @return true
 * @return false
 */
bool set_udp_server_config();

/**
 * @brief UDP服务端通信初始化，主要初始化收发队列（移到配置函数？）
 *
 * @param rx_queue
 * @param tx_queue
 * @return true
 * @return false
 */
bool udp_server_protocol_init(xQueueHandle *rx_queue, xQueueHandle *tx_queue);

/**
 * @brief 启动UDP数据接收任务
 * 
 * @return true 
 * @return false 
 */
bool udp_server_protocol_recv_task_init(void);

/**
 * @brief UDP服务端收发任务初始化
 *
 * @return true
 * @return false
 */
bool udp_server_protocol_task_init(void);

/**
 * @brief 根据配置连接服务端
 *
 * @return true
 * @return false
 */
bool udp_server_connect(void);


#endif  // _UDP_SERVER_PROTOCOL_H_
