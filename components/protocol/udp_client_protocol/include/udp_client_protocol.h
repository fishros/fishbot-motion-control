#ifndef _UDP_CLIENT_H_
#define _UDP_CLIENT_H_
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "proto_utils.h"

bool set_udp_client_config(mode_wifi_udp_pc_config_t *udp_pc_config_t);

bool udp_client_protocol_init(xQueueHandle *rx_queue, xQueueHandle *tx_queue);

bool udp_client_protocol_init();

bool udp_client_protocol_task_init(void);

/* 该模块需要根据wifi链接情况进行一些额外操作 */

bool update_udp_server_config();

#endif // _UDP_CLIENT_H_
