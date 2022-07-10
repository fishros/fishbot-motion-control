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

bool udp_client_init();
bool set_udp_client_config();
bool udp_client_send_data();

// void print_hex(char *buffer, int len);
// void my_udp_init();
// void send_data(char *data, uint8_t len);
// check sum fun
// uint8_t calc_checksum(char *p_data, int32_t data_len);
// static motor frame = {
//     .START=0x7D,
//     .TARGET = 0x01,
//     .CODE = 0X01,
//     .data_len = 0x04,
//     .spped_left = 0,
//     .spped_right = 0,
//     .sum = 0,
//     .END = 0x7E,
// };

// uint8_t calc_checksum(char *p_data, int32_t data_len)
// {
//     uint8_t sum = 0;
//     while (data_len--) {
//         sum += *p_data++;
//     }
//     return sum;
// }

// static void handleData(char* data_buff,uint8_t data_len)
// {
//     switch(data_buff[2])
//     {
//         case 0x01: memcpy(&frame,data_buff,data_len); break;
//     }
//     // print_hex((char*)&frame,sizeof(frame));
//     if(frame.sum==calc_checksum((char *)&frame+4,frame.data_len))
//     {
//         // ESP_LOGI(FISHBOT_MODULE, "speed_left:%d  spped_right:%d  ",frame.spped_left,frame.spped_right);
//         target_spped_left = frame.spped_left;
//         target_spped_right = frame.spped_right;
//     }
// }

#endif
