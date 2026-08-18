#include "L431_uart.h"
#include "main.h"        // 包含 HAL 库和 USART 句柄
#include "cmsis_os.h"          // 注意：这一行会包含 FreeRTOS.h 和 queue.h 等
#include "attention.h"   // 专注度计算函数
#include <stdio.h>
#include <string.h>
#include "queue.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

// 声明外部队列（在 freertos.c 中定义）
extern QueueHandle_t xUART_LineQueue;

// 数据接收缓存
#define PAIR_COUNT 256
//static uint16_t ch1_buf[PAIR_COUNT];
//static uint16_t ch2_buf[PAIR_COUNT];
//static uint16_t sample_cnt = 0;

// 串口行接收
static char rx_line[32];
static uint8_t rx_idx = 0;
static uint8_t rx_byte;

// 重定向 printf 到 USART1（调试）
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

//// 解析一行数据 "ch1,ch2"
//static void parse_line(char *line)
//{
//    uint16_t ch1, ch2;
//    if (sscanf(line, "%hu,%hu", &ch1, &ch2) == 2)
//    {
//        if (sample_cnt < PAIR_COUNT)
//        {
//            ch1_buf[sample_cnt] = ch1;
//            ch2_buf[sample_cnt] = ch2;
//            sample_cnt++;
//        }
//        if (sample_cnt == PAIR_COUNT)
//        {
//            // 计算专注度
//            uint8_t attention = compute_attention_from_adc(ch1_buf);
//            char msg[16];
//            sprintf(msg, "ATT:%d\n", attention);
//            HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);
//            printf("Attention = %d\n", attention);
//            sample_cnt = 0;
//        }
//    }
//}

//// 当攒够 256 对数据时，调用专注度计算（使用 CH1 数据）
//void process_data(void)
//{
//    uint8_t attention = compute_attention_from_adc(ch1_buf);
//    // 通过 USART3 发送结果到手机 APP（蓝牙）
//    char msg[16];
//    sprintf(msg, "ATT:%d\n", attention);
//    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);
//    // 调试打印
//    printf("Attention = %d\n", attention);
//}

// 串口接收中断回调（USART2 接收 L431 数据）
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        char c = rx_byte;
        if (c == '\n')
        {
            rx_line[rx_idx] = '\0';
            // 将整行发送到队列（注意：要从 ISR 中发送）
            if (xUART_LineQueue != NULL) {
                xQueueSendFromISR(xUART_LineQueue, rx_line, NULL);
            }
            rx_idx = 0;
        }
        else if (c != '\r')
        {
            if (rx_idx < sizeof(rx_line)-1)
                rx_line[rx_idx++] = c;
        }
        HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
    }
}

// 初始化串口接收（在 main 中调用）
void App_UART_Init(void)
{
    HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
}

