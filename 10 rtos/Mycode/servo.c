#include "servo.h"
#include "tim.h"   // 包含 htim1 句柄


#define SERVO_MIN_PULSE  300U   // 对应 0°300
#define SERVO_MAX_PULSE 1400U   // 对应 180°1450

/**
  * @brief 舵机初始化，启动 PWM 输出
  */
void Servo_Init(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    Servo_SetAngle(90);   // 转到中位（对应脉宽约 875）
    HAL_Delay(1000);
}

///**
//  * @brief 设置舵机角度
//  * @param angle 0~180
//  */
void Servo_SetAngle(uint8_t angle)
{
    if (angle > 180) angle = 180;
    uint16_t pulse = SERVO_MIN_PULSE + (uint32_t)angle * (SERVO_MAX_PULSE - SERVO_MIN_PULSE) / 180;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse);
}


// 平滑渐变到目标角度 (0~180)
void Servo_SmoothToAngle(uint8_t target_angle, uint16_t delay_ms)
{
    static uint8_t cur_angle = 90;  // 记录当前角度，初始为中间值

    // 限制目标角度在 0~180 之间
    if (target_angle > 180) target_angle = 180;

    // 递增
    if (cur_angle < target_angle) {
        while (cur_angle < target_angle) {
            cur_angle++;
            Servo_SetAngle(cur_angle);
            HAL_Delay(delay_ms);
        }
    }
    // 递减
    else if (cur_angle > target_angle) {
        while (cur_angle > target_angle) {
            cur_angle--;
            Servo_SetAngle(cur_angle);
            HAL_Delay(delay_ms);
        }
    }
}


