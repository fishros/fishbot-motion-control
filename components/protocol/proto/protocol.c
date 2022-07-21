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
      print_frame_to_hex((uint8_t *)"inver", frame,
                         frame_len);
      print_frame_to_hex((uint8_t *)"inver", frame + 3,
                         frame_len - 6);
      printf("frame_len=%d,data_cal_crc_=%d,data_crc_=%d\n", frame_len, data_cal_crc_, data_crc_);
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
        memcpy(&ctrl_data, frame + frame_parse_index,
               data_header.data_len);
        // call update speed
        proto_update_motor_speed(&ctrl_data);

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
    xQueueSend(data_tx_queue_, protocol_package, 2 / portTICK_RATE_MS);
    // 20ms加上串口延时，串口通信可以达到30HZ以上
    vTaskDelay(20 / portTICK_RATE_MS);
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
  xTaskCreate(proto_upload_data_task, "proto_upload_data_task", 1024 * 2, NULL,
              5, NULL); //发送任务
  xTaskCreate(proto_deal_data_task, "proto_deal_data_task", 1024 * 2, NULL, 5,
              NULL); //接收任务
  return true;
}