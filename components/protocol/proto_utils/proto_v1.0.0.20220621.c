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
    // crc16 data
    if (proto_motor_encoder_data_ == NULL)
    {
        protocol_package_.size = sprintf((char *)protocol_package_.data, "encoder:NULL!");
    }
    else
    {
        // compose
        protocol_package_.size = sprintf((char *)protocol_package_.data, "encoder%d!", proto_motor_encoder_data_->motor_encoder[0]);
    }
    // return
    *protocol_package = &protocol_package_;
    return (*protocol_package)->size;
}
