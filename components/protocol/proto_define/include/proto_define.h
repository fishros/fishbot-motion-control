#ifndef PROTO_DEFINE_H_
#define PROTO_DEFINE_H_

#define PROTO_V1_0_0_220806


#ifdef PROTO_V1_0_0_220621
#include "proto_v1.0.0.20220621.h"
#endif
#ifdef PROTO_V1_0_0_220806
#include "proto_v1.0.0.20220806.h"
#endif


/******************************应用协议重命名***************************************/
typedef proto_data_wifi_config_t fishbot_wifi_config_t;
typedef proto_data_pid_config_t fishbot_pid_config_t;
typedef proto_data_proto_mode_config_t fishbot_proto_config_t;


#endif // PROTO_DEFINE_H_