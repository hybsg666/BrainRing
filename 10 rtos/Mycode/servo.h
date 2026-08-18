#ifndef __SERVO_H
#define __SERVO_H

#include <stdint.h>

/**
  * @brief 初始化舵机（启动 PWM 输出）
  * @note  必须在 MX_TIM1_Init() 之后调用
  */
void Servo_Init(void);

/**
  * @brief 设置舵机角度
  * @param angle 目标角度，范围 0~180
  * @retval None
  */
void Servo_SetAngle(uint8_t angle);

void Servo_SmoothToAngle(uint8_t target_angle, uint16_t delay_ms);
	
#endif /* __SERVO_H */

