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

#define FIRST_CORE 0x5A
#define END_CORE 0x5A
#define TARGET_ADD 0x01 
/**
 * @brief crc16-XMODEM校验
 * 
 * @param buf 输入数组
 * @param len 数组长度
 * @return uint16_t 
 */
uint16_t crc16(unsigned char *buf, int len);

/**
 * @brief 设置传感器数据，处理成data数据，后续传入发送函数里
 * 
 * @param data_ 处理后的数组
 * @param encoder_vel 传入的左右编码器数据
 * @param imu_ 传入的IMU数据，MPU6050-三轴角速度&三轴加速度
 */
void set_sensor(unsigned char *data_,signed short int *encoder_vel,signed short int *imu_);

/**
 * @brief 设置控制速度数据，处理成data数据，后续传入发送函数里
 * 
 * @param data_ 处理后的数组
 * @param target_speed_ 传入的左右轮的数组
 */
void set_control_speed(unsigned char*data_ , signed short int *target_speed_);

/**
 * @brief 设置PID数据，处理成data数据，后续传入发送函数里
 * 
 * @param data_ 处理后的数组
 * @param pid_ p，i，d 三个数据组成的数组
 */
void set_pid(unsigned char*data_,signed short int*pid_);

/**
 * @brief 设置版本数据，处理成data数据，后续传入发送函数里
 * 
 * @param data_ 处理后的数组
 * @param ver_ 硬件版本和软件版本，两个数据组成的数组
 */
void set_version(unsigned char *data_ , signed short int*ver_);


/**
 * @brief 复制长度length_数组
 * 
 * @param data_ 复制的目标数组
 * @param tem 目标数组的副本
 * @param length_ 要复制数组的长度
 */
void copy_data(unsigned char*data_,unsigned char*tem,int length_);

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
void tx_task(void *arg);

/**
 * @brief 接收任务
 * 
 * @param arg 名字
 */
void rx_task(void *arg);

/**
 * @brief 串口协议的初始化
 * 
 */
void uart_protocol_init(void);
