#include "protocol.h"

#define FISHBOT_MODLUE "PROTOCOL"

static fishbot_proto_config_t *protocol_config_;

static xQueueHandle data_rx_queue_;
static xQueueHandle data_tx_queue_;

bool set_protocol_config(fishbot_proto_config_t *protocol_config)
{
  protocol_config_ = protocol_config;
  if (protocol_config->mode == PROTO_MODE_WIFI_UDP_CLIENT)
  {
    set_udp_client_config(&protocol_config);
  }
  return true;
}

bool protocol_init()
{
  /* 初始化队列 */
  data_rx_queue_ = xQueueCreate(5, sizeof(protocol_package_t));
  data_tx_queue_ = xQueueCreate(5, sizeof(protocol_package_t));
  /* 根据模式配置不同通信任务 */
  if (protocol_config_->mode == PROTO_MODE_UART)
  {
    uart_protocol_init(&data_rx_queue_, &data_tx_queue_);
  }
  else if (protocol_config_->mode == PROTO_MODE_WIFI_UDP_CLIENT)
  {
    udp_client_protocol_init(&data_rx_queue_, &data_tx_queue_);
  }
  else if (protocol_config_->mode == PROTO_MODE_WIFI_UDP_SERVER)
  {
    udp_server_protocol_init(&data_rx_queue_, &data_tx_queue_);
  }

  return true;
}

void proto_deal_data_task(void *params)
{
  static protocol_package_t protocol_package; // 待处理

  static uint8_t frame[RX_TX_PACKET_SIZE]; // 原始数据帧
  static uint16_t frame_len;               // 原始数据帧长度
  static uint16_t frame_parse_index;       // 解析数据时临时索引
  /*数据帧相关*/
  static uint8_t frame_index;                  // 数据帧编号
  static uint16_t data_crc_;                   // 数据中的CRC
  static uint16_t data_cal_crc_;               // 计算数据域得出的CRC
  static proto_data_target_addr_t target_addr; // 数据帧目标地址
  /*数据域相关*/
  static uint8_t data_frame_size;         // 数据帧中数据个数
  static proto_data_header_t data_header; // 解析数据时临时索引
  /*被解析的数据定义*/
  static proto_motor_speed_ctrl_data_t ctrl_data; // 电机速度控制数据
  static proto_data_device_control_t ctrl_device; // 设备控制数据
  static proto_data_wifi_config_t wifi_config;    // WIFI配置数据

  while (true)
  {
    if (xQueueReceive(data_rx_queue_, &protocol_package, 5) == pdTRUE)
    {
      frame_parse_index = 1;

      // 反转义
      frame_len = inverse_escape_frame((uint8_t *)protocol_package.data, frame,
                                       protocol_package.size);

      frame_index = frame[frame_parse_index++];
      target_addr = frame[frame_parse_index++];
      // 获取数据中的CRC值
      SET_SUB_BYTES1(data_crc_, frame[frame_len - 2]);
      SET_SUB_BYTES2(data_crc_, frame[frame_len - 3]);
      // 计算数据域的CRC值
      data_cal_crc_ = crc16(frame + 3, frame_len - 6);
#ifdef DEBUG_FISHBOT
      print_frame_to_hex((uint8_t *)"recv", protocol_package.data,
                         protocol_package.size);
      print_frame_to_hex((uint8_t *)"inver", frame, frame_len);
      print_frame_to_hex((uint8_t *)"inver", frame + 3, frame_len - 6);
      printf("frame_len=%d,data_cal_crc_=%d,data_crc_=%d\n", frame_len,
             data_cal_crc_, data_crc_);
#endif
      // 校验CRC
      if (data_cal_crc_ != data_crc_)
      {
        print_frame_to_hex((uint8_t *)"error frame", frame, frame_len);
        continue;
      }
      // 解析数据域
      data_frame_size = frame[frame_parse_index++];       // 数据个数
      memcpy(&data_header, frame + frame_parse_index, 4); // 获取数据头
      frame_parse_index += 4;
#ifdef DEBUG_FISHBOT
      printf("recv data header id=%d len=%d\n", data_header.data_id,
             data_header.data_len);
#endif
      switch (data_header.data_id)
      {
      case DATA_SPEED:
        // 收到速度控制数据
        memcpy(&ctrl_data, frame + frame_parse_index, data_header.data_len);
        // 调用速度更新函数
        proto_update_motor_speed(&ctrl_data);
        break;
      case DATA_DEV_CONTROL:
        memcpy(&ctrl_device, frame + frame_parse_index, data_header.data_len);
        if (ctrl_device.operation == DEIVCE_RESTART)
        {
          // 如果是重启指令则重新启动设备
          esp_restart();
        }
        break;
      case DATA_WIFI_CONFIG:
        memcpy(&wifi_config, frame + frame_parse_index, data_header.data_len);
        proto_update_wifi_config(&wifi_config);
        break;

#ifdef DEBUG_FISHBOT
        printf("recv speed[%d,%d]\n", ctrl_data.motor_speed[0],
               ctrl_data.motor_speed[1]);
#endif
        break;

      default:
        break;
      }
      // call fun update
    }
  }
}

void proto_upload_data_task(void *params)
{
  static protocol_package_t *protocol_package;
  while (true)
  {
    proto_get_upload_frame(&protocol_package);
    if (xQueueSend(data_tx_queue_, protocol_package, 2 / portTICK_RATE_MS) !=
        pdTRUE)
    {
      ESP_LOGW(FISHBOT_MODLUE, "send to queue failed!");
    }
    // 20ms:串口通信可以达到30HZ以上
    vTaskDelay(20 / portTICK_RATE_MS);
  }
}

bool protocol_task_init(void)
{
  // 根据模式启动不同通信任务
  if (protocol_config_->mode == PROTO_MODE_UART)
  {
    uart_protocol_task_init();
  }
  else if (protocol_config_->mode == PROTO_MODE_WIFI_UDP_CLIENT)
  {
    udp_client_protocol_task_init();
  }
  else if (protocol_config_->mode == PROTO_MODE_WIFI_UDP_SERVER)
  {
    udp_server_protocol_task_init();
  }
  // 启动数据帧收发任务
  xTaskCreate(proto_upload_data_task, "proto_upload_data_task", 1024 * 2, NULL,
              8, NULL); //发送任务
  xTaskCreate(proto_deal_data_task, "proto_deal_data_task", 1024 * 2, NULL, 8,
              NULL); //接收任务
  return true;
}

bool update_protocol_config(fishbot_proto_config_t *protocol_config)
{
  // 如果发生了状态改变，主要是服务器IP地址改变了，需要进行更新
  // 第一阶段是否可以直接将地址写入到NVS中，如果需要更新，插入USB线，发送更新数据帧（暂未定义），重启即可
  // 下一版本数据帧需要加入，重启指令、FishBotREADY指令、电量状态反馈、电机配置指令、编码器清空指令、
  return true;
}