#include "TOF400C.h"


extern TIM_HandleTypeDef htim1;

// 定义 GetDutyByDistance 函数
static uint8_t GetDutyByDistance(int32_t distance_mm)
{
    if (distance_mm >= 300) return 20;   // 弱
    else if (distance_mm <= 100) return 100; // 强
    else return 60;                      // 中
}

// 声明外部变量（定义在 vl53l1x.c 中）
extern VL53L1_Dev_t VL53;
extern int32_t distance;

void TOF400C_Init(void)
{
	 if (getDistance(&VL53) == VL53L1_ERROR_NONE)
    {
        // 根据距离获取目标占空比
        uint8_t duty = GetDutyByDistance(distance);
        
        // 设置 PWM 占空比（ARR = 100，直接赋值）
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty);
        
        // 打印距离和占空比（可选，便于调试）
        printf("Distance = %ld mm, Duty = %d%%\r\n", distance, duty);
    }
    else
    {
        // 测距失败，停止震动
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        printf("Measurement error\r\n");
    }
    
    HAL_Delay(100);  // 控制测距频率
}

void TOF400C_getmotor(void)
{
		   // 调用测距函数，返回值 VL53L1_ERROR_NONE 表示成功
    if (getDistance(&VL53) == VL53L1_ERROR_NONE)
    {
        // 根据当前距离计算目标占空比（0~100）
        uint8_t duty = GetDutyByDistance(distance);
        
        // 设置 TIM1 通道 1 的比较值（CCR），从而改变占空比
        // ARR = 100，所以 CCR = duty 时，占空比 = duty%
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty);
        
        // 打印距离和占空比，便于调试
        printf("Distance = %ld mm, Duty = %d%%\r\n", distance, duty);
    }
    else
    {
        // 测距失败（例如传感器未就绪或通信错误）
        // 停止马达震动，并将错误提示打印出来
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        printf("Measurement error\r\n");
    }
}
