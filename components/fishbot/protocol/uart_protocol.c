
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

static const int RX_BUF_SIZE = 1024;  //串口缓存帧的大小

uint8_t data_count = 0;          //帧序

//这些数据用于传入函数
signed short int set_target_speed_[2] = {0x5325,0x5436};
signed short int encoder_vel[2] = {0x5041,0x3135};
signed short int imu_[6]={0x0000,0x4564,0x0987,0x0000,0x505A,0x5035};
signed short int set_pid_[3]={0x0425,0x5358,0x4567};
signed short int set_ver_[2]={0x0557,0x0824};

//--接收帧中的数据定义
uint8_t rece_data_count = 0;   //接收帧中的帧序
uint8_t rece_target_add = 0x01; //接收帧的地址
uint16_t rece_crc16_data;       //接收帧的crc16校验
uint8_t rece_data_operation = 0; //接收帧中的操作码
signed short int rece_target_speed_[2]; 
signed short int rece_encoder_vel[2];
signed short int rece_imu_[6];
signed short int rece_pid_[3];
signed short int rece_ver_[2];
//-----------------
//------------------------crc16校验-------
static const uint16_t crc16tab[256]= {
    0x0000,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,
    0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef,
    0x1231,0x0210,0x3273,0x2252,0x52b5,0x4294,0x72f7,0x62d6,
    0x9339,0x8318,0xb37b,0xa35a,0xd3bd,0xc39c,0xf3ff,0xe3de,
    0x2462,0x3443,0x0420,0x1401,0x64e6,0x74c7,0x44a4,0x5485,
    0xa56a,0xb54b,0x8528,0x9509,0xe5ee,0xf5cf,0xc5ac,0xd58d,
    0x3653,0x2672,0x1611,0x0630,0x76d7,0x66f6,0x5695,0x46b4,
    0xb75b,0xa77a,0x9719,0x8738,0xf7df,0xe7fe,0xd79d,0xc7bc,
    0x48c4,0x58e5,0x6886,0x78a7,0x0840,0x1861,0x2802,0x3823,
    0xc9cc,0xd9ed,0xe98e,0xf9af,0x8948,0x9969,0xa90a,0xb92b,
    0x5af5,0x4ad4,0x7ab7,0x6a96,0x1a71,0x0a50,0x3a33,0x2a12,
    0xdbfd,0xcbdc,0xfbbf,0xeb9e,0x9b79,0x8b58,0xbb3b,0xab1a,
    0x6ca6,0x7c87,0x4ce4,0x5cc5,0x2c22,0x3c03,0x0c60,0x1c41,
    0xedae,0xfd8f,0xcdec,0xddcd,0xad2a,0xbd0b,0x8d68,0x9d49,
    0x7e97,0x6eb6,0x5ed5,0x4ef4,0x3e13,0x2e32,0x1e51,0x0e70,
    0xff9f,0xefbe,0xdfdd,0xcffc,0xbf1b,0xaf3a,0x9f59,0x8f78,
    0x9188,0x81a9,0xb1ca,0xa1eb,0xd10c,0xc12d,0xf14e,0xe16f,
    0x1080,0x00a1,0x30c2,0x20e3,0x5004,0x4025,0x7046,0x6067,
    0x83b9,0x9398,0xa3fb,0xb3da,0xc33d,0xd31c,0xe37f,0xf35e,
    0x02b1,0x1290,0x22f3,0x32d2,0x4235,0x5214,0x6277,0x7256,
    0xb5ea,0xa5cb,0x95a8,0x8589,0xf56e,0xe54f,0xd52c,0xc50d,
    0x34e2,0x24c3,0x14a0,0x0481,0x7466,0x6447,0x5424,0x4405,
    0xa7db,0xb7fa,0x8799,0x97b8,0xe75f,0xf77e,0xc71d,0xd73c,
    0x26d3,0x36f2,0x0691,0x16b0,0x6657,0x7676,0x4615,0x5634,
    0xd94c,0xc96d,0xf90e,0xe92f,0x99c8,0x89e9,0xb98a,0xa9ab,
    0x5844,0x4865,0x7806,0x6827,0x18c0,0x08e1,0x3882,0x28a3,
    0xcb7d,0xdb5c,0xeb3f,0xfb1e,0x8bf9,0x9bd8,0xabbb,0xbb9a,
    0x4a75,0x5a54,0x6a37,0x7a16,0x0af1,0x1ad0,0x2ab3,0x3a92,
    0xfd2e,0xed0f,0xdd6c,0xcd4d,0xbdaa,0xad8b,0x9de8,0x8dc9,
    0x7c26,0x6c07,0x5c64,0x4c45,0x3ca2,0x2c83,0x1ce0,0x0cc1,
    0xef1f,0xff3e,0xcf5d,0xdf7c,0xaf9b,0xbfba,0x8fd9,0x9ff8,
    0x6e17,0x7e36,0x4e55,0x5e74,0x2e93,0x3eb2,0x0ed1,0x1ef0
};

uint16_t crc16(unsigned char *buf, int len) {
    int counter;
    uint16_t crc = 0;
    for (counter = 0; counter < len; counter++)
            crc = (crc<<8) ^ crc16tab[((crc>>8) ^ *buf++)&0x00FF];
    return crc;
}


//--------------------协议一--------------------
// 编码器（左右轮count数量）与IMU数据（MPU6050-三轴角速度&三轴加速度）
void set_sensor(unsigned char*data_,signed short int*encoder_vel,signed short int *imu_)
{
  data_[0] = 0x02;//数据总量
  data_[1] = 0x01;//数据编号
  data_[2] = 0x00;//数据长度高位
  data_[3] = 0x04;//数据长度低位
  data_[4] = 0x01;//数据操作
  data_[5] = encoder_vel[0] >> 8;//左轮count高位
  data_[6] = encoder_vel[0];//左轮count低位
  data_[7] = encoder_vel[1] >> 8;//右轮count高位
  data_[8] = encoder_vel[1] ;//右轮count低位
  data_[9] = 0x02;//数据编号2
  data_[10] = 0x00;//编号2数据长度高位
  data_[11] = 0x0c;//编号2数据长度低位
  data_[12] = 0x01;//数据操作
  //IMU的三轴角速度和加速度 高位在前，低位在后
  data_[13] = imu_[0] >> 8;
  data_[14] = imu_[0];
  data_[15] = imu_[1] >> 8;
  data_[16] = imu_[1];
  data_[17] = imu_[2] >> 8;
  data_[18] = imu_[2];
  data_[19] = imu_[3] >> 8;
  data_[20] = imu_[3];
  data_[21] = imu_[4] >> 8;
  data_[22] = imu_[4];
  data_[23] = imu_[5] >> 8;
  data_[24] = imu_[5];

}

//--------------------协议二--------------------
// 左右电机速度控制
void set_control_speed(unsigned char*data_ , signed short int *target_speed_)
{ 
  data_[0] = 0x01; //数据总数
  data_[1] = 0x03; //数据编号
  data_[2] = 0x00; //数据长度高位
  data_[3] = 0x04; //数据长度低位
  data_[4] = 0x02; //数据操作
  data_[5] = target_speed_[0] >> 8; //左电机高位
  data_[6] = target_speed_[0]; //左电机低位
  data_[7] = target_speed_[1] >> 8; //右电机高位
  data_[8] = target_speed_[1]; //右电机低位
 
  }

//--------------------协议三--------------------
// PID参数设置
void set_pid(unsigned char*data_,signed short int*pid_)
{
  data_[0] = 0x01;  //数据总数
  data_[1] = 0x04;  //数据编号
  data_[2] = 0x00;  //数据长度位
  data_[3] = 0x06;  //数据长度低位
  data_[4] = 0x01;  //数据操作，1反馈数据（底盘向主控），2命令数据（主控向底盘）
  data_[5] = pid_[0] >> 8;  //p高位 
  data_[6] = pid_[0];  //p低位
  data_[7] = pid_[1] >> 8;  //i高位
  data_[8] = pid_[1];  //i低位
  data_[9] = pid_[2] >> 8;  //d高位 
  data_[10] = pid_[2]; //d低位 
 
  }

//--------------------协议四--------------------
// 版本，软硬件版本以及ASCII明文传输
void set_version(unsigned char *data_ , signed short int*ver_)
{
  
  data_[0] = 0x01; //数据总数
  data_[1] = 0x05; //数据编号
  data_[2] = 0x00; //数据长度高位
  data_[3] = 0x04; //数据长度低位
  data_[4] = 0x01; //数据操作，1反馈数据（底盘向主控），2命令数据（主控向底盘）
  data_[5] = ver_[0] >> 8; //硬件版本高位
  data_[6] = ver_[0]; //硬件版本低位
  data_[7] = ver_[1] >> 8; //软件版本高位
  data_[8] = ver_[1]; //软件版本低位
  
  }

//---------------发送函数
int send_data_function(const char* logName,unsigned char*send_data_,int length_)
{
    static uint8_t send_data[100];
    static uint8_t tem[100];
    static int change_number = 0;//转义次数
    
    uint16_t crc16_data;           //crc校验位，高位在前，低位在后
    send_data[0] = (unsigned char)FIRST_CORE;
    send_data[1] = (unsigned char)data_count++;
    send_data[2] = (unsigned char)TARGET_ADD;
    for(int i = 0;i < length_;i++)
    {
      send_data[3+i] = (unsigned char)send_data_[i];
    }
    crc16_data = crc16(send_data,length_+3);
    send_data[3+length_] = (unsigned char)(crc16_data >> 8);
    send_data[4+length_] = (unsigned char)(crc16_data) ;
    
    //-----------转义
    //0x5A转义成0x50 0x0A，除0x50 0x0A出现0x50，使用0x50,0x05替代
    copy_data(send_data,tem,length_+5);
    for(int j = 1;j<length_+5+change_number;j++)
    {
      if((send_data[j]==0x5A))
      {
        change_number ++;
        send_data[j] = 0x50;
        j++;
        send_data[j] = 0x0A;
        for(int n = j+1;n<length_+5+change_number;n++)
        {
          send_data[n] = tem[n-change_number]; 
          }
        }
      else if((send_data[j] == 0x50 ) &&(send_data[j+1] != 0x0A))
      {
        change_number++;
        send_data[j] = 0x50;
        j++;
        send_data[j] = 0x05;
        for(int n = j+1;n<length_+5+change_number;n++)
          {
            send_data[n] = tem[n-change_number];
          }
        }
    }
    send_data[length_+5+change_number] = END_CORE;
    const int txBytes = uart_write_bytes(UART_NUM_1, send_data, (length_+5+change_number+1));                         
    ESP_LOGI(logName, "Wrote %d bytes", txBytes+1);
    change_number = 0;

     return txBytes;
  }

void copy_data(unsigned char*data_,unsigned char*tem,int length_)
{
  for(int i = 0;i<length_;i++)
  {
    tem[i] = data_[i]; 
  }
}


void handle_case(const char* logName ,unsigned char *data_,int length_)
{
  static unsigned char tem[100];
  tem[0] = 0x5A;
  for(int i = 0;i<length_-2;i++)
  {
    tem[i+1] = data_[i];
  }
  rece_crc16_data = crc16(tem,length_-1);
  //------校验正确-----
  //可以根据length_长度判断类型
  //13-----version  版本 or  速度控制
  //15-----PID数据
  //29-----传感器数据
  if(((rece_crc16_data >> 8) == data_[length_-2]) && ((unsigned char)rece_crc16_data == data_[length_-1]))
  {
    rece_data_count = data_[0];
    rece_target_add = data_[1];
    switch(length_)
    {
      case 13:
        if(data_[3] == 0x05)
        {
          rece_data_operation = data_[6];
          rece_ver_[0] = (data_[7] << 8) | data_[8];
          rece_ver_[1] = (data_[9] << 8) | data_[10];
          //版本信息
          ESP_LOGI(logName, "ver[0]:%d  ver[1]:%d", rece_ver_[0],rece_ver_[1]);
        }
        else if(data_[3] == 0x03)
        {
          rece_data_operation = data_[6];
          rece_target_speed_[0] = (data_[7] << 8) | data_[8];
          rece_target_speed_[1] = (data_[9] << 8) | data_[10];
          //电机速度
          ESP_LOGI(logName, "vel[0]:%d  vel[1]:%d", rece_target_speed_[0],rece_target_speed_[1]);
        }
        break;
      case 15:
        //PID
          rece_data_operation = data_[6];
          rece_pid_[0] = (data_[7] << 8) | data_[8];
          rece_pid_[1] = (data_[9] << 8) | data_[10];
          rece_pid_[2] = (data_[11] << 8) | data_[12]; 
          ESP_LOGI(logName, "p:%d  i:%d  d:%d", rece_pid_[0],rece_pid_[1],rece_pid_[2]);
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
          ESP_LOGI(logName, "re-vel[0]:%d  re-vel[1]:%d", rece_encoder_vel[0],rece_encoder_vel[1]);      
          ESP_LOGI(logName, "imu[0]:%d  imu[1]:%d  imu[2]:%d", rece_imu_[0],rece_imu_[1],rece_imu_[2]);
          ESP_LOGI(logName, "imu[3]:%d  imu[4]:%d  imu[5]:%d", rece_imu_[3],rece_imu_[4],rece_imu_[5]);    
        break;
      
    }
    
  }
}


//接收函数
void rece_data_function(const char* logName)
{
  static int count = 0;
  static int number = 0;
  uint8_t tem[100] = {0};
  uint8_t rece_buf[100]; //接收数组
  static int change_number = 0;
  int res;
//---------接收部分，0x5A开始，0x5A结束
//---------存放在rece_buf数组中，后续以rece_buf作为处理的中心
  uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
  const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 20 / portTICK_RATE_MS);
  
  if(rxBytes > 0)
  {
       ESP_LOGI(logName, "Read %d bytes: '%s'", rxBytes, data);  //串口日志
      //ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
      for(int ii = 0;ii<rxBytes;ii++)
        {
            res = data[ii];    
            switch(number)
            {
                case 0:
                    if(res == 0x5A) number++;
                    else number = 0;
                    break;
                case 1:
                if(res == 0x5A)
                {
                    number = 0;
        //----------------逆转义-------
        //------转义规则----0x5A转义成0x50 0x0A，除0x50 0x0A出现0x50，使用0x50,0x05替代
        //------逆转义规则----0x50 0x0A 组成0x5A，0x50 0x05组成0x50
                    copy_data(rece_buf,tem,count);
                    for(int j = 0;j<count-change_number;j++)
                        {
                            if((rece_buf[j] == 0x50) && (rece_buf[j+1] == 0x0A) )
                                {
                                    change_number++;
                                    rece_buf[j] = 0x5A;
                                    for(int n =j+1;n<count;n++)
                                        {
                                        rece_buf[n] = tem[n+change_number];
                                        } 
                                    }
                            else if((rece_buf[j] == 0x50) && (rece_buf[j+1] == 0x05))
                                {
                                        change_number++;
                                        rece_buf[j] = 0x50;
                                    for(int n =j+1;n<count;n++)   
                                    {
                                        rece_buf[n] = tem[n+change_number];
                                    }
                                }
                            }
                    handle_case(logName,rece_buf,count-change_number);
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

//uart串口初始化
void uart_init(void) {
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

int sendData(const char* logName, unsigned char* data,int len)
{
    const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
    ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}

void tx_task(void *arg)
{
    static const char *TX_TASK_TAG = "TX_TASK";
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
    while (1) {

        uint8_t ver_data[9];  
        uint8_t pid_data[11];
        uint8_t vel_data[9];
        uint8_t sensor_data[25];
        set_version(ver_data,set_ver_);  //发送版本信息
        set_pid(pid_data,set_pid_);  //发送PID信息
        set_control_speed(vel_data,set_target_speed_); //发送目标速度信息
        set_sensor(sensor_data,encoder_vel,imu_);//发送传感器信息
        
        send_data_function(TX_TASK_TAG, ver_data,sizeof(ver_data));
        vTaskDelay(50 / portTICK_PERIOD_MS);  //20延时
    }
}

//读取操作
void rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    while (1) {
        rece_data_function(RX_TASK_TAG);
    }
}

void uart_protocol_init(void)
{
    uart_init();
    xTaskCreate(rx_task, "uart_rx_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);  //接收任务
    xTaskCreate(tx_task, "uart_tx_task", 1024*2, NULL, configMAX_PRIORITIES-1, NULL);  //发送任务
}
