/**
 * @brief 文件描述：待更新
 * @author 小鱼 (fishros@foxmail.com)
 * @version V1.0.0
 * @date 2022-07-07
 * @copyright 版权所有：FishBot Open Source Organization
 */
#ifndef _OLED_H_
#define _OLED_H_
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp32_i2c_rw.h"

#define OLED_ADDR 0x3C /*!< slave address for OLED sensor */

typedef struct
{
    uint8_t x;
    uint8_t y;
    uint8_t row[128];
} oled_show_line_t;

/**
 * @brief OLED初始化
 *
 * @return true
 * @return false
 */
bool oled_init(void);

/**
 * @brief 清除屏幕
 *
 */
void oled_clear(void);

/**
 * @brief 设置坐标
 * 0-----------128>x
 * |
 * |
 * 8
 * y
 * @param x
 * @param y
 */
void oled_setxy(uint8_t x, uint8_t y);

/**
 * @brief 自动的一行行显示，自动覆盖
 * 
 * @param len 字符串长度
 * @return true 
 * @return false 
 */
bool oled_show_ascii_auto_line(char *str);

/**
 * @brief 写ASSIC字符
 *
 * @param x
 * @param y
 * @param str
 */
void oled_ascii(uint8_t x, uint8_t y, char *str);

#endif