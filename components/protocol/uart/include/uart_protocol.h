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


#define TXD_PIN (GPIO_NUM_4)
#define RXD_PIN (GPIO_NUM_5)
