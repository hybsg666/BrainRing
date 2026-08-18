#ifndef __L431_UART_H
#define __L431_UART_H

#include <stdint.h>

void App_UART_Init(void);   // 初始化串口相关（启动中断接收等）
// 注意：printf 重定向不需要额外声明

#endif

