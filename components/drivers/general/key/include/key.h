#ifndef _KEY_H__
#define _KEY_H__
#include <time.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "protocol.h"


#define GPIO_INPUT_IO 0


/**
 * @brief 
 * 
 */
void key_init(void);

/**
 * @brief 
 * 
 */
void key_task_init(void);


#endif