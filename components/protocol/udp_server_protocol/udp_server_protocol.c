#include "udp_server_protocol.h"

#define FISHBOT_MODULE "UDP_SERVER"
static const char *TAG = FISHBOT_MODULE;

#define TX_BUF_SIZE 256  // UDP TX缓存帧的大小
#define RX_BUF_SIZE 256  // UDP RX缓存帧的大小

static xQueueHandle *rx_queue_;
static xQueueHandle *tx_queue_;
static char frame_buffer_tx_[TX_BUF_SIZE];
static char frame_buffer_rx_[RX_BUF_SIZE];
static protocol_package_t frame_pack_tx_;
static protocol_package_t frame_pack_rx_;

#define PORT 3474
// #define DEBUG_FISHBOT

typedef enum {
  UDP_CONNECT_STATUS_FALSE,
  UDP_CONNECT_STATUS_TRUE,
} udp_client_connect_status_t;

static int8_t is_udp_status_ok = UDP_CONNECT_STATUS_FALSE;  // UDP连接化状态
static struct sockaddr_in dest_addr;                        // 目标地址
static struct sockaddr_storage source_addr;                 // 当前地址
static socklen_t socklen = sizeof(source_addr);             // 地址长度
static int sock = -1;                                       // Server的SocketID

bool udp_server_connect(void) {
  if (sock != -1) {
    ESP_LOGE(TAG, "Shutting down socket and restarting...");
    shutdown(sock, 0);
    close(sock);
  }
  struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
  dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
  dest_addr_ip4->sin_family = AF_INET;
  dest_addr_ip4->sin_port = htons(PORT);
  sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
  if (sock < 0) {
    ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
    is_udp_status_ok = UDP_CONNECT_STATUS_FALSE;
    return;
  }
  ESP_LOGI(TAG, "Socket created");
  int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
  if (err < 0) {
    ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
  }
  ESP_LOGI(TAG, "Socket bound, port %d", PORT);
  is_udp_status_ok = UDP_CONNECT_STATUS_TRUE;
  return true;
}

static void udp_server_tx_task(void *parameters) {
  static int16_t tx_bytes_len = 0;
  static uint16_t frame_index_ = 0;

  while (true) {
    if (is_udp_status_ok == UDP_CONNECT_STATUS_FALSE) {
      vTaskDelay(200);
      udp_server_connect();  // 这里检测连接状态，出现问题立即重连
      continue;
    }
    if (xQueueReceive(*tx_queue_, &frame_pack_tx_, 5) == pdTRUE) {
      tx_bytes_len =
          sendto(sock, frame_pack_tx_.data, frame_pack_tx_.size, 0,
                 (struct sockaddr *)&source_addr, sizeof(source_addr));
      frame_index_++;
#ifdef DEBUG_FISHBOT
      printf("send frame %d \n", tx_bytes_len);
#endif
      if (tx_bytes_len < 0) {
      }
    }
  }
}

static void udp_server_rx_task(void *parameters) {
  static uint16_t i = 0;
  static uint16_t rx_index = 0;
  static int16_t frame_start_index = -1;
  static int16_t frame_end_index = -1;
  uint16_t rx_bytes_len;
  while (true) {
    if (is_udp_status_ok == UDP_CONNECT_STATUS_FALSE) {
      vTaskDelay(20);
      continue;
    }
    rx_bytes_len = recvfrom(sock, frame_buffer_rx_, RX_BUF_SIZE, 0,
                            (struct sockaddr *)&source_addr, &socklen);
    if (rx_bytes_len < 0) {
      is_udp_status_ok = UDP_CONNECT_STATUS_FALSE;
      continue;
    }

    // 处理数据,将数据分解为一帧帧
    frame_start_index = -1;
    frame_end_index = -1;
#ifdef DEBUG_FISHBOT
    print_frame_to_hex((uint8_t *)"rxraw",
                       (uint8_t *)frame_buffer_rx_ + rx_index, rx_bytes_len);
    printf("rx_index=%d,rx_bytes_len=%d\n", rx_index, rx_bytes_len);
#endif
    for (i = 0; i < rx_index + rx_bytes_len; i++) {
      if (frame_buffer_rx_[i] == 0x5A) {
        if (frame_start_index == -1) {
          frame_start_index = i;
        } else {
          frame_end_index = i;
          if (frame_end_index - frame_start_index == 1) {
            frame_start_index = frame_end_index;
            frame_end_index = -1;
          }
        }
      }
#ifdef DEBUG_FISHBOT
      printf("start=%d,end=%d,i=%d\n", frame_start_index, frame_end_index, i);
#endif
      if (frame_end_index != -1 && frame_start_index != -1) {
        frame_pack_rx_.size = frame_end_index - frame_start_index + 1;
        memcpy(frame_pack_rx_.data, frame_buffer_rx_, frame_pack_rx_.size);
        xQueueSend(*rx_queue_, &frame_pack_rx_, 2 / portTICK_RATE_MS);
        frame_end_index = -1;
        frame_start_index = -1;
      }
    }
    if (frame_start_index != -1) {
      rx_index = rx_index + rx_bytes_len - frame_start_index;
      memcpy(frame_buffer_rx_, frame_buffer_rx_ + frame_start_index, rx_index);
    }
    frame_start_index = -1;
    frame_end_index = -1;
  }
}

bool set_udp_server_config() { return true; }

bool udp_server_protocol_init(xQueueHandle *rx_queue, xQueueHandle *tx_queue) {
  rx_queue_ = rx_queue;
  tx_queue_ = tx_queue;
  udp_server_connect();  // 初始化时先尝试连接一次
  return true;
}

bool udp_server_protocol_task_init(void) {
  xTaskCreate(udp_server_tx_task, "udp_server_tx_task", 1024 * 2, NULL, 7,
              NULL);
  xTaskCreate(udp_server_rx_task, "udp_server_rx_task", 1024 * 2, NULL, 7,
              NULL);
  return true;
}

bool udp_server_protocol_recv_task_init(void) {
  xTaskCreate(udp_server_rx_task, "udp_server_rx_task", 1024 * 2, NULL, 7,
              NULL);
  return true;
}
