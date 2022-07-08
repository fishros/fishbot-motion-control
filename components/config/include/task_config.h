/**
 * @brief 文件描述：待更新
 * @author 小鱼 (fishros@foxmail.com)
 * @version V1.0.0
 * @date 2022-07-07
 * @copyright 版权所有：FishBot Open Source Organization
 */
#ifndef _TASK_CONFIG_H_
#define _TASK_CONFIG_H_

// // if task watchdog triggered,KALMAN_TASK_PRI should set lower or set lower flow frequency
// #ifdef TARGET_MCU_ESP32
// #define KALMAN_TASK_PRI 2
// #define LOG_TASK_PRI 1
// #define MEM_TASK_PRI 1
// #define PARAM_TASK_PRI 1
// #endif

#define SYSLINK_TASK_PRI 3
#define USBLINK_TASK_PRI 3
#define ACTIVE_MARKER_TASK_PRI 3
#define AI_DECK_TASK_PRI 3
#define UART2_TASK_PRI 3
#define WIFILINK_TASK_PRI 3
#define UDP_TX_TASK_PRI 3
#define UDP_RX_TASK_PRI 3
#define UDP_RX2_TASK_PRI 3

// Task names
#define SYSTEM_TASK_NAME "SYSTEM"
#define LEDSEQCMD_TASK_NAME "LEDSEQCMD"
#define ADC_TASK_NAME "ADC"
#define UART_RX_TASK_NAME "UART"
#define ACTIVE_MARKER_TASK_NAME "ACTIVEMARKER-DECK"
#define AI_DECK_GAP_TASK_NAME "AI-DECK-GAP"
#define AI_DECK_NINA_TASK_NAME "AI-DECK-NINA"
#define UART2_TASK_NAME "UART2"

#define configBASE_STACK_SIZE 512

// Task stack sizes
#define SYSTEM_TASK_STACKSIZE (6 * configBASE_STACK_SIZE)
#define LEDSEQCMD_TASK_STACKSIZE (2 * configBASE_STACK_SIZE)
#define ADC_TASK_STACKSIZE (1 * configBASE_STACK_SIZE)

#endif /* _TASK_CONFIG_H_ */
