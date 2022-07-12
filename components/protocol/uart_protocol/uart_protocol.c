
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

#define UART_USB_NUM UART_NUM_1
#define UART_LOG_NUM UART_NUM_0

#define UART_PROTOC_NUM UART_USB_NUM


#define TX_BUF_SIZE 256 // 串口缓存帧的大小
#define RX_BUF_SIZE 256 // 串口缓存帧的大小

static xQueueHandle *rx_queue_;
static xQueueHandle *tx_queue_;
static bool is_uart_init_ = false;
static char frame_buffer_tx_[TX_BUF_SIZE];
static char frame_buffer_rx_[RX_BUF_SIZE];
static protocol_package_t frame_pack_tx_;
static protocol_package_t frame_pack_rx_;

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
    // 发送数据用CP2101
    uart_driver_install(UART_USB_NUM, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_USB_NUM, &uart_config);
    uart_set_pin(UART_USB_NUM, TXD_CP2102_PIN, RXD_CP2102_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    // 日志放到日志另外引脚打印出来
    uart_driver_install(UART_LOG_NUM, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_LOG_NUM, &uart_config);
    uart_set_pin(UART_LOG_NUM, TXD_LOG_PIN, RXD_LOG_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

bool uart_protocol_init(xQueueHandle *rx_queue, xQueueHandle *tx_queue)
{
    rx_queue_ = rx_queue;
    tx_queue_ = tx_queue;
    // uart hardware init
    uart_init();
    is_uart_init_ = true;
    return is_uart_init_;
}

static void uart_tx_task(void *pvParameters)
{
    static int16_t tx_bytes_len = 0;
    while (true)
    {
        if (is_uart_init_ == false)
        {
            vTaskDelay(20);
            continue;
        }
        if (xQueueReceive(*tx_queue_, &frame_pack_tx_, 5) == pdTRUE)
        {
            tx_bytes_len = uart_write_bytes(UART_PROTOC_NUM, frame_pack_tx_.data, frame_pack_tx_.size);
            if (tx_bytes_len <= 0)
            {
                // TODO() Add Error LOG!
            }
#ifdef DEBUG_UART
            print_hex();
#endif
        }
    }
}

static void uart_rx_task(void *pvParameters)
{
    static uint16_t i = 0;
    static uint16_t rx_index = 0;
    static int16_t frame_start_index = -1;
    static int16_t frame_end_index = -1;
    uint16_t rx_bytes_len;
    while (true)
    {
        if (is_uart_init_ == false)
        {
            vTaskDelay(20);
            continue;
        }

        rx_bytes_len = uart_read_bytes(UART_PROTOC_NUM, frame_buffer_rx_ + rx_index, RX_BUF_SIZE, 10 / portTICK_RATE_MS);

        if (rx_bytes_len <= 0)
        {
            continue;
        }

        // 处理数据
        for (i = 0; i < rx_index + rx_bytes_len; i++)
        {
            if (frame_buffer_rx_[i] == 0x5A)
            {
                if (frame_start_index == -1)
                {
                    frame_start_index = i;
                }
                else
                {
                    frame_end_index = i;
                }
            }

            if (frame_end_index != -1 && frame_start_index != -1)
            {
                frame_pack_rx_.size = frame_end_index - frame_start_index + 1;
                memcpy(frame_pack_rx_.data, frame_buffer_rx_, frame_pack_rx_.size);
                xQueueSend(*rx_queue_, &frame_pack_rx_, 2 / portTICK_RATE_MS);
                frame_end_index = -1;
                frame_start_index = -1;
            }
        }
        if (frame_start_index != -1)
        {
            rx_index = rx_index + rx_bytes_len - frame_start_index;
            memcpy(frame_buffer_rx_, frame_buffer_rx_ + frame_start_index, rx_index);
        }
        frame_start_index = -1;
        frame_end_index = -1;
    }
}

bool uart_protocol_task_init(void)
{
    xTaskCreate(uart_rx_task, "uart_rx_task", 1024 * 2, NULL, 5, NULL); //接收任务
    xTaskCreate(uart_tx_task, "uart_tx_task", 1024 * 2, NULL, 4, NULL); //发送任务
    return true;
}