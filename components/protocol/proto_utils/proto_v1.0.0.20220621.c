#include "proto_v1.0.0.20220621.h"

update_pid_params_fun_t update_pid_params_fun_;

bool proto_register_update_pid_fun(update_pid_params_fun_t *update_pid_params_fun)
{
    update_pid_params_fun = update_pid_params_fun;
    return true;
}

uint16_t proto_deal_frame_data(protocol_package_t *protocol_package)
{
    static uint8_t frame[RX_TX_PACKET_SIZE];
    static uint16_t frame_size = 0;
    frame_size = inverse_escape_frame(protocol_package->data, frame, protocol_package->size);
    print_frame_to_hex((uint8_t *)"uart frame", frame, frame_size);
    return protocol_package->size;
}

static protocol_package_t protocol_package_;

uint16_t proto_get_upload_frame(protocol_package_t **protocol_package)
{
    // update data 

    // crc16 data

    // compose
    sprintf((char *)protocol_package_.data, "This is a log frame!");
    protocol_package_.size = 21;
    // return
    *protocol_package = &protocol_package_;
    return (*protocol_package)->size;
}
