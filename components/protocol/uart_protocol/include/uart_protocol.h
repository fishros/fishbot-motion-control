/**
 * @file uart_protocol.h
 * @author cuizhongren (1326986768@qq.com)
 * @brief
 * @version 0.1
 * @date 2022-07-08
 *
 * 版权所有：FishBot Open Source Organization
 *
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "proto_utils.h"

#define TXD_LOG_PIN (GPIO_NUM_17)
#define RXD_LOG_PIN (GPIO_NUM_16)

#define TXD_CP2102_PIN (GPIO_NUM_1)
#define RXD_CP2102_PIN (GPIO_NUM_3)


bool uart_protocol_init(xQueueHandle *rx_queue, xQueueHandle *tx_queue);

bool uart_protocol_task_init(void);
