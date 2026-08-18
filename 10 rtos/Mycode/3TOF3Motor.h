#ifndef __3TOF3MOTOR_H
#define __3TOF3MOTOR_H

#include "vl53l1.h"
#include <stdint.h>

// 三个传感器的 I2C 7位地址
#define TOF_ADDR_LEFT   0x30

#define TOF_ADDR_RIGHT  0x32

// XSHUT 引脚定义
#define XSHUT_LEFT_PORT   GPIOE
#define XSHUT_LEFT_PIN    GPIO_PIN_0      // 左前方传感器使用 PE0


#define XSHUT_RIGHT_PORT  GPIOE
#define XSHUT_RIGHT_PIN   GPIO_PIN_1      // 右前方传感器使用 PE1

// 全局设备句柄和距离变量
extern VL53L1_Dev_t tof_left, tof_right;
extern int32_t dist_left, dist_right;

// 初始化所有传感器（修改地址、启动测量）
void MultiTOF_Init(void);

// 轮询读取所有传感器的距离（非阻塞）
void MultiTOF_Update(void);

// 根据距离设置指定马达的占空比 (channel: 1=左,2=前,3=右)
void SetMotorDutyByDistance(int32_t dist, uint8_t channel);


#endif

