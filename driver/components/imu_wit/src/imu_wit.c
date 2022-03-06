/*
 * @作者: 小鱼
 * @公众号: 鱼香ROS
 * @QQ交流群: 2642868461
 * @描述: IMU相关
 */
#include"imu_wit.h"


#define ECHO_TEST_TXD 17
#define ECHO_TEST_RXD 16
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)

#define ECHO_UART_PORT_NUM      2
#define ECHO_UART_BAUD_RATE     9600
#define ECHO_TASK_STACK_SIZE    2048

#define BUF_SIZE (256)

char YAWCMD[3] = {0XFF,0XAA,0X52};
char ACCCMD[3] = {0XFF,0XAA,0X67};


static void uart_init()
{
    uart_config_t uart_config = {
        .baud_rate = ECHO_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    int intr_alloc_flags = 0;
    ESP_ERROR_CHECK(uart_driver_install(ECHO_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(ECHO_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(ECHO_UART_PORT_NUM, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));
}


static void imu_task(void *arg)
{
    uart_init();
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    uint8_t read_len = 0;
    uint8_t i = 0 ;

    imu_t frame = {
        .START=0x7D,
        .TARGET = 0x01,
        .CODE = 0X02,
        .data_len = 0x12,
        .sum = 0,
        .END = 0x7E,
    };

    // init imu
    uart_write_bytes(ECHO_UART_PORT_NUM, (const char *) ACCCMD, 3);
    vTaskDelay(pdMS_TO_TICKS(100));
    uart_write_bytes(ECHO_UART_PORT_NUM, (const char *) YAWCMD, 3);
    vTaskDelay(pdMS_TO_TICKS(100));

    while (1) {
        read_len = uart_read_bytes(ECHO_UART_PORT_NUM, data, BUF_SIZE, 20 / portTICK_RATE_MS);


        if (read_len<33) continue;

        for (i=0;i<read_len;i++)
        {
            if(data[i]==0x55)
            {
                // printf("detect,0x55,i:%d\n",i);
                switch(data[i+1])//判断数据是哪种数据，然后将其拷贝到对应的结构体中，有些数据包需要通过上位机打开对应的输出后，才能接收到这个数据包的数据
                {
                    //memcpy为编译器自带的内存拷贝函数，需引用"string.h"，将接收缓冲区的字符拷贝到数据结构体里面，从而实现数据的解析。
                    case 0x51:	memcpy(frame.sacc,&data[i+2],6); i+=10;break;
                    case 0x52:	memcpy(frame.sgy,&data[i+2],6);i+=10;break;
                    case 0x53:	memcpy(frame.sangle,&data[i+2],6);i+=10;break;
                }
            }
        }
        // 组装发送给server
        frame.sum =  calc_checksum((char *)&frame+4,frame.data_len);
        send_data((char *)&frame,sizeof(frame));
        // print_hex((char *)&frame,sizeof(frame));
        // printf("check sum:%d",frame.sum);
        
        vTaskDelay(pdMS_TO_TICKS(30));
    }

}


void my_imu_init()
{
    xTaskCreate(imu_task, "imu_task", ECHO_TASK_STACK_SIZE, NULL, 10, NULL);
}
