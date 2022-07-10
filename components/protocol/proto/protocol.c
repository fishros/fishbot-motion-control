#include "protocol.h"

static protocol_config_t *protocol_config_;

bool set_protocol_config(protocol_config_t *protocol_config)
{
  protocol_config_ = protocol_config;
  return true;
}

bool protocol_init()
{
  if(protocol_config_->mode==MODE_USB)
  {
    // uart init
  }else if(protocol_config_->mode==MODE_WIFI_UDP_PC)
  {
    // udp_client_init
  }
  return true;
}