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


/**
 * @brief 串口发送数据帧
 * 
 * @param logName esp32日志对象名
 * @param send_data_ 要发送的数据，传感器、速度、pid和版本中的一个
 * @param length_ 要发送数据的长度
 * @return int 返回值是发送数据的长度
 */
int send_data_function(const char* logName,unsigned char*send_data_,int length_);

/**
 * @brief 逆转义的处理
 * 
 * @param logName esp32日志对象名
 * @param data_ 逆转义对象数组
 * @param length_ 逆转义对象数据长度
 */
void handle_case(const char* logName,unsigned char *data_,int length_);

/**
 * @brief 接收串口数据，并处理成传感器、速度、pid、版本信息中的一个
 * 
 * @param logName esp32日志对象名
 */
void rece_data_function(const char* logName );


/**
 * @brief 串口初始化
 * 
 */
void uart_init(void) ;

/**
 * @brief 串口发送函数，这个函数是send_data_function的基础
 * 
 * @param logName  esp32日志对象名
 * @param data  要发送的数据
 * @param len  要发送的数据长度
 * @return int 返回值是发送数据的长度
 */
int sendData(const char* logName, unsigned char* data,int len);


/**
 * @brief 发送任务
 * 
 * @param arg 名字
 */
void uart_tx_task(void *arg);

/**
 * @brief 接收任务
 * 
 * @param arg 名字
 */
void uart_rx_task(void *arg);

/**
 * @brief 串口协议的初始化
 * 
 */
void uart_protocol_init(void);
