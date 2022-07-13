#ifndef _UDP_CLIENT_H_
#define _UDP_CLIENT_H_

#define CONFIG_EXAMPLE_IPV4 y
#define PORT 3333
#define KEEPALIVE_IDLE 5
#define KEEPALIVE_INTERVAL 5
#define KEEPALIVE_COUNT 3

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

#include <string.h>

bool set_udp_client_config();
bool udp_client_protocol_init();
bool udp_client_protocol_task_init();

/* 该模块需要根据wifi链接情况进行一些额外操作 */
/* 结构体可直接强转：(char *)&frame*/

#endif // _UDP_CLIENT_H_
