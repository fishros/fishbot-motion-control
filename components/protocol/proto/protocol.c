#include "protocol.h"

#define FISHBOT_MODLUE "PROTOCOL"

static protocol_config_t *protocol_config_;

static xQueueHandle data_rx_queue_;
static xQueueHandle data_tx_queue_;

bool set_protocol_config(protocol_config_t *protocol_config)
{
  protocol_config_ = protocol_config;
  return true;
}

bool protocol_init()
{
  /* 初始化队列 */
  data_rx_queue_ = xQueueCreate(5, sizeof(protocol_config_t));
  data_tx_queue_ = xQueueCreate(5, sizeof(protocol_config_t));
  /* 根据模式配置不同通信任务 */
  if (protocol_config_->mode == MODE_USB)
  {
    uart_protocol_init(&data_rx_queue_, &data_tx_queue_);
  }
  else if (protocol_config_->mode == MODE_WIFI_UDP_PC)
  {
    // udp_client_init
  }

  return true;
}

bool protocol_task_init(void)
{
  // 根据模式启动不同通信任务
  if (protocol_config_->mode == MODE_USB)
  {
    uart_protocol_task_init();
  }
  else if (protocol_config_->mode == MODE_WIFI_UDP_PC)
  {
    // udp_client_init
  }
  // start self task
  return true;
}

void deal_recv_data()
{
  // get a frame
  // crccheck
  // target div
  // call fun update
}

void deal_send_data()
{
  // gen a frame
  // crc16 
  // send data
}