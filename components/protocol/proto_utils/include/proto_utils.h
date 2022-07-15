#ifndef _PROTO_UTILS_H_
#define _PROTO_UTILS_H_
#include "freertos/FreeRTOS.h"

#define RX_TX_PACKET_SIZE (128)

#define FIRST_CODE 0x5A
#define END_CODE 0x5A
#define TARGET_ADD 0x01

/* Structure used for in/out data via USB */
typedef struct
{
    uint8_t size;
    uint8_t data[RX_TX_PACKET_SIZE];
} protocol_package_t;

/**
 * @brief crc16-XMODEM校验
 *
 * @param buf 输入数组
 * @param len 数组长度
 * @return uint16_t
 */
uint16_t crc16(uint8_t *buf, int len);

/**
 * @brief 将数据进行转义
 *
 * @param frame 帧数据
 * @param result 结果
 * @param len 长度
 * @return int 转义完成后新的帧的大小
 */
int escape_frame(uint8_t *frame, uint8_t *result, int len);

/**
 * @brief 将数据帧进行反转义
 *
 * @param frame 帧数据
 * @param result 结果
 * @param len 长度
 * @return int
 */
int inverse_escape_frame(uint8_t *frame, uint8_t *result, int len);

/**
 * @brief 将数据帧打印成hex形式
 *
 * @param title 数据标题
 * @param buffer 数据
 * @param size 长度
 */
void print_frame_to_hex(uint8_t *title, uint8_t *buffer, uint16_t size);

#endif // _PROTO_UTILS_H_