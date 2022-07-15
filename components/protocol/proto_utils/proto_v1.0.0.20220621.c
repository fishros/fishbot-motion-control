#include "proto_v1.0.0.20220621.h"

static update_pid_params_fun_t update_pid_params_fun_;
static protocol_package_t protocol_package_;
static proto_motor_encoder_data_t *proto_motor_encoder_data_;

bool proto_register_update_pid_fun(update_pid_params_fun_t *update_pid_params_fun)
{
    update_pid_params_fun_ = update_pid_params_fun;
    return true;
}

void proto_set_motor_encoder_data(proto_motor_encoder_data_t *proto_motor_encoder_data)
{
    proto_motor_encoder_data->data_id = DATA_ENCODER;
    proto_motor_encoder_data->data_direction = DATA_TO_MASTER;
    proto_motor_encoder_data->data_len = sizeof(proto_motor_encoder_data_t) - 4; // 不计算id、direction、data_len空间
    proto_motor_encoder_data_ = proto_motor_encoder_data;
}

uint16_t proto_deal_frame_data(protocol_package_t *protocol_package)
{
    static uint8_t frame[RX_TX_PACKET_SIZE];
    static uint16_t frame_size = 0;
    frame_size = inverse_escape_frame(protocol_package->data, frame, protocol_package->size);
    print_frame_to_hex((uint8_t *)"uart frame", frame, frame_size);
    return protocol_package->size;
}

uint16_t proto_get_upload_frame(protocol_package_t **protocol_package)
{
    // update data
    static uint8_t frame_data[RX_TX_PACKET_SIZE]; // 数据域
    static uint8_t frame[RX_TX_PACKET_SIZE+20];      // 原始帧数据
    static uint8_t frame_index = 0;               // 帧序列
    static uint16_t crc_result = 0;               // crc16校验结果
    static uint16_t frame_len = 0;
    // 编码器数据
    frame_len = sprintf((char *)frame_data, "\x01%s", (char *)proto_motor_encoder_data_);
    // 对数据域进行CRC16校验，生成校验码
    crc_result = crc16(frame_data, frame_len - 1); //结尾应该被多补了一个NULL
    frame_len = sprintf((char*)frame,"%c%c.000%c%s%d%c",FIRST_CODE,frame_index++,DATA_TARGET_ADDR_PC,frame_data,crc_result,END_CODE);
    // 对数据帧进行转义
    protocol_package_.size = escape_frame(frame,protocol_package_.data,frame_len);
    *protocol_package = &protocol_package_;
    return (*protocol_package)->size;
}
