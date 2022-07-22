#include "udp_client_protocol.h"

#define TX_BUF_SIZE 256 // 串口缓存帧的大小
#define RX_BUF_SIZE 256 // 串口缓存帧的大小

static xQueueHandle *rx_queue_;
static xQueueHandle *tx_queue_;
static bool is_uart_init_ = false;
static char frame_buffer_tx_[TX_BUF_SIZE];
static char frame_buffer_rx_[RX_BUF_SIZE];
static protocol_package_t frame_pack_tx_;
static protocol_package_t frame_pack_rx_;

#define CONFIG_EXAMPLE_IPV4 y
#define PORT 3333
#define KEEPALIVE_IDLE 5
#define KEEPALIVE_INTERVAL 5
#define KEEPALIVE_COUNT 3

#define FISHBOT_MODULE "UDP_CLIENT"

#define HOST_IP_ADDR "192.168.4.2"
#define PORT 3474

// uint8_t send_error = 0;
static int8_t is_udp_status_ok = -1;

static struct sockaddr_in dest_addr;            // 目标地址
static struct sockaddr_storage source_addr;     // 当前地址
static socklen_t socklen = sizeof(source_addr); // 地址长度
static int sock = -1;


bool set_udp_client_config(mode_wifi_udp_pc_config_t *udp_pc_config_t);

bool udp_client_protocol_init(xQueueHandle *rx_queue, xQueueHandle *tx_queue)
{
    rx_queue_ = rx_queue;
    tx_queue_ = tx_queue;
    // udp_client_protocol_init();
    return true;
}

bool udp_client_protocol_task_init(void)
{
    // xTaskCreate(udp_client_task, "upd_client", 4096, NULL, 5, NULL);
    // xTaskCreate(udp_client_task, "upd_client", 4096, NULL, 5, NULL);
    return true;
}


bool udp_client_init()
{
    dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT);
    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0)
    {
        ESP_LOGE(FISHBOT_MODULE, "Unable to create socket: errno %d", errno);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        // break;
        return false;
    }

    ESP_LOGI(FISHBOT_MODULE, "Socket created, sending to %s:%d", HOST_IP_ADDR, PORT);
    // ioctlsocket(sock, FIONBIO, &non_blocking);
    //设置为非阻塞
    // setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    return true;
}

static void udp_client_tx_task(void *parameters)
{
    static int16_t tx_bytes_len = 0;
    while (true)
    {
        if (is_udp_status_ok == false)
        {
            vTaskDelay(20);
            continue;
        }
        if (xQueueReceive(*tx_queue_, &frame_pack_tx_, 5) == pdTRUE)
        {
            // tx_bytes_len = uart_write_bytes(UART_PROTOC_NUM, frame_pack_tx_.data, frame_pack_tx_.size);
            // tx_bytes_len = sendto(sock, data, len, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (tx_bytes_len < 0)
            {
                // TODO() Add Error LOG!
            }
        }
    }
}

static void udp_client_rx_task(void *parameters)
{
   while (true)
   {
    // len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);
    //             if(send_error){
    //                 shutdown(sock, 0);
    //                 close(sock);
    //                 break;
    //             }
       vTaskDelay(10 / portTICK_PERIOD_MS);
   }
}

// static void udp_client_task(void *pvParameters)
// {
//     char rx_buffer[128];
//     int addr_family = 0;
//     int ip_protocol = 0;
//     uint32_t non_blocking=1;
//     struct timeval timeout={
//         .tv_sec = 0,
//         .tv_usec = 20000,
//     };

//     while (1) {
//         dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
//         dest_addr.sin_family = AF_INET;
//         dest_addr.sin_port = htons(PORT);
//         addr_family = AF_INET;
//         ip_protocol = IPPROTO_IP;
//         sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
//         if (sock < 0) {
//             ESP_LOGE(FISHBOT_MODULE, "Unable to create socket: errno %d", errno);
//             vTaskDelay(2000 / portTICK_PERIOD_MS);
//             // break;
//             continue;
//         }
//         ESP_LOGI(FISHBOT_MODULE, "Socket created, sending to %s:%d", HOST_IP_ADDR, PORT);

//         ioctlsocket(sock,FIONBIO,&non_blocking);
//         //设置为非阻塞
//         setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
//         send_error = 0;

//         struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
//         socklen_t socklen = sizeof(source_addr);
//         int len=0 ;
//         while (1) {
//             len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);
//             if (len >= 0) {
//                 // rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
//                 handleData(rx_buffer,len);
//             }
//             if(send_error){
//                 shutdown(sock, 0);
//                 close(sock);
//                 break;
//             }
//             vTaskDelay(10 / portTICK_PERIOD_MS);
//         }

//         if (sock != -1) {
//             ESP_LOGE(FISHBOT_MODULE, "Shutting down socket and restarting...");
//             shutdown(sock, 0);
//             close(sock);
//         }
//     }
//     vTaskDelete(NULL);
// }

// void my_udp_init()
// {
//     xTaskCreate(udp_client_task, "upd_client", 4096, NULL, 5, NULL);
// }
