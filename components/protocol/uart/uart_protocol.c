
/**
 * @file uart_protocol.c
 * @author cuizhongren (1326986768@qq.com)
 * @brief
 * @version 0.1
 * @date 2022-07-08
 *
 * 版权所有：FishBot Open Source Organization
 *
 */
#include "uart_protocol.h"

static const int RX_BUF_SIZE = 1024; //串口缓存帧的大小

//---------------发送函数
int send_data_function(const char *logName, unsigned char *send_data_, int length_)
{
  static uint8_t send_data[100];
  static uint8_t tem[100];
  static int change_number = 0; //转义次数

  uint16_t crc16_data; // crc校验位，高位在前，低位在后
  send_data[0] = (unsigned char)FIRST_CORE;
  send_data[1] = (unsigned char)data_count++;
  send_data[2] = (unsigned char)TARGET_ADD;
  for (int i = 0; i < length_; i++)
  {
    send_data[3 + i] = (unsigned char)send_data_[i];
  }
  crc16_data = crc16(send_data, length_ + 3);
  send_data[3 + length_] = (unsigned char)(crc16_data >> 8);
  send_data[4 + length_] = (unsigned char)(crc16_data);

  //-----------转义
  // 0x5A转义成0x50 0x0A，除0x50 0x0A出现0x50，使用0x50,0x05替代
  copy_data(send_data, tem, length_ + 5);
  for (int j = 1; j < length_ + 5 + change_number; j++)
  {
    if ((send_data[j] == 0x5A))
    {
      change_number++;
      send_data[j] = 0x50;
      j++;
      send_data[j] = 0x0A;
      for (int n = j + 1; n < length_ + 5 + change_number; n++)
      {
        send_data[n] = tem[n - change_number];
      }
    }
    else if ((send_data[j] == 0x50) && (send_data[j + 1] != 0x0A))
    {
      change_number++;
      send_data[j] = 0x50;
      j++;
      send_data[j] = 0x05;
      for (int n = j + 1; n < length_ + 5 + change_number; n++)
      {
        send_data[n] = tem[n - change_number];
      }
    }
  }
  send_data[length_ + 5 + change_number] = END_CORE;
  const int txBytes = uart_write_bytes(UART_NUM_1, send_data, (length_ + 5 + change_number + 1));
  ESP_LOGI(logName, "Wrote %d bytes", txBytes + 1);
  change_number = 0;

  return txBytes;
}

void copy_data(unsigned char *data_, unsigned char *tem, int length_)
{
  for (int i = 0; i < length_; i++)
  {
    tem[i] = data_[i];
  }
}

void handle_case(const char *logName, unsigned char *data_, int length_)
{
  static unsigned char tem[100];
  tem[0] = 0x5A;
  for (int i = 0; i < length_ - 2; i++)
  {
    tem[i + 1] = data_[i];
  }
  rece_crc16_data = crc16(tem, length_ - 1);
  //------校验正确-----
  //可以根据length_长度判断类型
  // 13-----version  版本 or  速度控制
  // 15-----PID数据
  // 29-----传感器数据
  if (((rece_crc16_data >> 8) == data_[length_ - 2]) && ((unsigned char)rece_crc16_data == data_[length_ - 1]))
  {
    rece_data_count = data_[0];
    rece_target_add = data_[1];
    switch (length_)
    {
    case 13:
      if (data_[3] == 0x05)
      {
        rece_data_operation = data_[6];
        rece_ver_[0] = (data_[7] << 8) | data_[8];
        rece_ver_[1] = (data_[9] << 8) | data_[10];
        //版本信息
        ESP_LOGI(logName, "ver[0]:%d  ver[1]:%d", rece_ver_[0], rece_ver_[1]);
      }
      else if (data_[3] == 0x03)
      {
        rece_data_operation = data_[6];
        rece_target_speed_[0] = (data_[7] << 8) | data_[8];
        rece_target_speed_[1] = (data_[9] << 8) | data_[10];
        //电机速度
        ESP_LOGI(logName, "vel[0]:%d  vel[1]:%d", rece_target_speed_[0], rece_target_speed_[1]);
      }
      break;
    case 15:
      // PID
      rece_data_operation = data_[6];
      rece_pid_[0] = (data_[7] << 8) | data_[8];
      rece_pid_[1] = (data_[9] << 8) | data_[10];
      rece_pid_[2] = (data_[11] << 8) | data_[12];
      ESP_LOGI(logName, "p:%d  i:%d  d:%d", rece_pid_[0], rece_pid_[1], rece_pid_[2]);
      break;
    case 29:
      //传感器数据
      rece_data_operation = data_[6];
      rece_encoder_vel[0] = (data_[7] << 8) | data_[8];
      rece_encoder_vel[1] = (data_[9] << 8) | data_[10];
      rece_imu_[0] = (data_[15] << 8) | data_[16];
      rece_imu_[1] = (data_[17] << 8) | data_[18];
      rece_imu_[2] = (data_[19] << 8) | data_[20];
      rece_imu_[3] = (data_[21] << 8) | data_[22];
      rece_imu_[4] = (data_[23] << 8) | data_[24];
      rece_imu_[5] = (data_[25] << 8) | data_[26];
      ESP_LOGI(logName, "re-vel[0]:%d  re-vel[1]:%d", rece_encoder_vel[0], rece_encoder_vel[1]);
      ESP_LOGI(logName, "imu[0]:%d  imu[1]:%d  imu[2]:%d", rece_imu_[0], rece_imu_[1], rece_imu_[2]);
      ESP_LOGI(logName, "imu[3]:%d  imu[4]:%d  imu[5]:%d", rece_imu_[3], rece_imu_[4], rece_imu_[5]);
      break;
    }
  }
}

//接收函数
void rece_data_function(const char *logName)
{
  static int count = 0;
  static int number = 0;
  uint8_t tem[100] = {0};
  uint8_t rece_buf[100]; //接收数组
  static int change_number = 0;
  int res;
  //---------接收部分，0x5A开始，0x5A结束
  //---------存放在rece_buf数组中，后续以rece_buf作为处理的中心
  uint8_t *data = (uint8_t *)malloc(RX_BUF_SIZE + 1);
  const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 20 / portTICK_RATE_MS);

  if (rxBytes > 0)
  {
    ESP_LOGI(logName, "Read %d bytes: '%s'", rxBytes, data); //串口日志
    // ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
    for (int ii = 0; ii < rxBytes; ii++)
    {
      res = data[ii];
      switch (number)
      {
      case 0:
        if (res == 0x5A)
          number++;
        else
          number = 0;
        break;
      case 1:
        if (res == 0x5A)
        {
          number = 0;
          //----------------逆转义-------
          //------转义规则----0x5A转义成0x50 0x0A，除0x50 0x0A出现0x50，使用0x50,0x05替代
          //------逆转义规则----0x50 0x0A 组成0x5A，0x50 0x05组成0x50
          copy_data(rece_buf, tem, count);
          for (int j = 0; j < count - change_number; j++)
          {
            if ((rece_buf[j] == 0x50) && (rece_buf[j + 1] == 0x0A))
            {
              change_number++;
              rece_buf[j] = 0x5A;
              for (int n = j + 1; n < count; n++)
              {
                rece_buf[n] = tem[n + change_number];
              }
            }
            else if ((rece_buf[j] == 0x50) && (rece_buf[j + 1] == 0x05))
            {
              change_number++;
              rece_buf[j] = 0x50;
              for (int n = j + 1; n < count; n++)
              {
                rece_buf[n] = tem[n + change_number];
              }
            }
          }
          handle_case(logName, rece_buf, count - change_number);
          count = 0;
          change_number = 0;
          break;
        }
        else
        {
          rece_buf[count] = res;
          count++;
        }
        break;
      }
    }
  }
  free(data);
}

// uart串口初始化
void uart_init(void)
{
  const uart_config_t uart_config = {
      .baud_rate = 115200,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_APB,
  };
  // We won't use a buffer for sending data.
  uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
  uart_param_config(UART_NUM_1, &uart_config);
  uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

int sendData(const char *logName, unsigned char *data, int len)
{
  const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
  ESP_LOGI(logName, "Wrote %d bytes", txBytes);
  return txBytes;
}

void uart_tx_task(void *arg)
{
  static const char *TX_TASK_TAG = "TX_TASK";
  esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
  while (1)
  {

    uint8_t ver_data[9];
    uint8_t pid_data[11];
    uint8_t vel_data[9];
    uint8_t sensor_data[25];
    set_version(ver_data, set_ver_);                //发送版本信息
    set_pid(pid_data, set_pid_);                    //发送PID信息
    set_control_speed(vel_data, set_target_speed_); //发送目标速度信息
    set_sensor(sensor_data, encoder_vel, imu_);     //发送传感器信息

    send_data_function(TX_TASK_TAG, ver_data, sizeof(ver_data));
    vTaskDelay(50 / portTICK_PERIOD_MS); // 20延时
  }
}

//读取操作
void uart_rx_task(void *arg)
{
  static const char *RX_TASK_TAG = "RX_TASK";
  esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
  while (1)
  {
    rece_data_function(RX_TASK_TAG);
  }
}

void uart_protocol_init(void)
{
  uart_init();
  xTaskCreate(uart_rx_task, "uart_rx_task", 1024 * 2, NULL, configMAX_PRIORITIES, NULL);     //接收任务
  xTaskCreate(uart_tx_task, "uart_tx_task", 1024 * 2, NULL, configMAX_PRIORITIES - 1, NULL); //发送任务
}
