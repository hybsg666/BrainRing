#include "adc_poll.h"

// 简单的轮询采集函数（放在这里，避免依赖外部文件）
void adc_poll_init(void)
{
    // 无需额外初始化，MX_ADC1_Init 已完成
}

void adc_poll_collect(uint16_t *buffer, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        HAL_ADC_Start(&hadc1);
        if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
            buffer[i] = (uint16_t)HAL_ADC_GetValue(&hadc1);
        } else {
            buffer[i] = 0;
        }
        HAL_ADC_Stop(&hadc1);
        // 简单延时约 4ms（80MHz 下约 10000 次循环），可调整
        for (volatile int d = 0; d < 10000; d++);
    }
}


