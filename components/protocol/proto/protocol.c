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

void deal_recv_data()
{
  // get a frame
  // crccheck
  // target div
  // call fun update
}

void proto_upload_data_task(void *param)
{
  static protocol_package_t *protocol_package;
  while (true)
  {
    proto_get_upload_frame(&protocol_package);
    xQueueSend(data_tx_queue_,protocol_package, 2 / portTICK_RATE_MS);
    vTaskDelay(1000 / portTICK_RATE_MS);
  }
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
  xTaskCreate(proto_upload_data_task, "proto_upload_data_task", 1024 * 2, NULL, 5, NULL); //接收任务
  return true;
}