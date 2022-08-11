#ifndef _PROTOCOL_H_
#define _PROTOCOL_H_
#include "freertos/FreeRTOS.h"

#include "proto_utils.h"
#include "proto_define.h"
#include "udp_client_protocol.h"
#include "udp_server_protocol.h"
#include "uart_protocol.h"

/**
 * @brief 设置协议模块配置
 * 
 * @param protocol_config* 配置指针 
 * @return true 
 * @return false 
 */
bool set_protocol_config(fishbot_proto_config_t *protocol_config);

/**
 * @brief 更新通信配置
 * 
 * @param protocol_config 
 * @return true 
 * @return false 
 */
bool update_protocol_config(fishbot_proto_config_t *protocol_config);

/**
 * @brief 协议模块初始化
 * 
 * @return true 
 * @return false 
 */
bool protocol_init(void);

/**
 * @brief 协议任务初始化
 * 
 * @return true 
 * @return false 
 */
bool protocol_task_init(void);

#endif // _PROTOCOL_H_
