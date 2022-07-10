#ifndef _PROTO_UTILS_H_
#define _PROTO_UTILS_H_
#include "freertos/FreeRTOS.h"

/**
 * @brief crc16-XMODEM校验
 *
 * @param buf 输入数组
 * @param len 数组长度
 * @return uint16_t
 */
uint16_t crc16(unsigned char *buf, int len);

/**
 * @brief 发送数据前调用的转义函数
 *
 * @param frame 未转义的数据帧
 */
void escape_frame(char *frame);

/**
 * @brief 接收到数据后调用的反转义函数
 *
 * @param frame 待反转义的数据帧
 */
void inverse_escape_frame(char *frame);

#endif // _PROTO_UTILS_H_