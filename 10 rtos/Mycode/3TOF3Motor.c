#include "3TOF3Motor.h"
#include "vl53l1_api.h"
#include "main.h"
#include <stdio.h>
#include "i2c.h"
#include "tim.h"

extern TIM_HandleTypeDef htim3;

VL53L1_Dev_t tof_left, tof_right;
int32_t dist_left = 0, dist_right = 0;

// 修改传感器地址（I2C 句柄为 hi2c1）
static uint8_t SetSensorAddress(uint8_t old_7bit_addr, uint8_t new_7bit_addr)
{
    printf("SetSensorAddress: 0x%02X -> 0x%02X\r\n", old_7bit_addr, new_7bit_addr);
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Write(&hi2c1, (old_7bit_addr << 1), 0x0001,
                                              I2C_MEMADD_SIZE_16BIT, &new_7bit_addr, 1, 100);
    if (ret != HAL_OK) {
        printf("Write address failed, ret=%d\r\n", ret);
        return 0;
    }
    printf("Write success, waiting...\r\n");
    HAL_Delay(20);
    
    uint8_t id = 0;
    ret = HAL_I2C_Mem_Read(&hi2c1, (new_7bit_addr << 1), 0x010F, I2C_MEMADD_SIZE_16BIT, &id, 1, 100);
    if (ret == HAL_OK && id == 0xEA) {
        printf("Address changed successfully, ID=0x%02X\r\n", id);
        return 1;
    } else {
        printf("Verify failed, ret=%d, id=0x%02X\r\n", ret, id);
        return 0;
    }
}

// 初始化单个传感器（传入地址）
static uint8_t InitSensor(VL53L1_Dev_t* pDev, uint8_t i2c_addr)
{
    pDev->I2cDevAddr = i2c_addr;
    pDev->comms_type = 1;
    pDev->comms_speed_khz = 400;
    
    VL53L1_Error status;
    status = VL53L1_DataInit(pDev);
    if (status) { printf("DataInit failed, err=%d\r\n", status); return 0; }
    status = VL53L1_StaticInit(pDev);
    if (status) { printf("StaticInit failed, err=%d\r\n", status); return 0; }
    status = VL53L1_SetDistanceMode(pDev, VL53L1_DISTANCEMODE_LONG);
    if (status) { printf("SetDistanceMode failed, err=%d\r\n", status); return 0; }
    status = VL53L1_StartMeasurement(pDev);
    if (status) { printf("StartMeasurement failed, err=%d\r\n", status); return 0; }
    return 1;
}

// 初始化两个 TOF 传感器
void MultiTOF_Init(void)
{
    printf("MultiTOF_Init: start\r\n");
    
    // 复位左右传感器
    HAL_GPIO_WritePin(XSHUT_LEFT_PORT, XSHUT_LEFT_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(XSHUT_RIGHT_PORT, XSHUT_RIGHT_PIN, GPIO_PIN_RESET);
    for (uint32_t i = 0; i < 40000; i++) { __NOP(); }
    printf("Both XSHUT low\r\n");
    
    // 初始化左传感器
    HAL_GPIO_WritePin(XSHUT_LEFT_PORT, XSHUT_LEFT_PIN, GPIO_PIN_SET);
    for (uint32_t i = 0; i < 200000; i++) { __NOP(); }
    printf("Left XSHUT high, changing address...\r\n");
    if (!SetSensorAddress(0x29, TOF_ADDR_LEFT)) {
        printf("Left address change failed\r\n");
    } else {
        printf("Left address changed to 0x%02X\r\n", TOF_ADDR_LEFT);
    }
    
    // 初始化右传感器
    HAL_GPIO_WritePin(XSHUT_RIGHT_PORT, XSHUT_RIGHT_PIN, GPIO_PIN_SET);
    for (uint32_t i = 0; i < 200000; i++) { __NOP(); }
    printf("Right XSHUT high, changing address...\r\n");
    if (!SetSensorAddress(0x29, TOF_ADDR_RIGHT)) {
        printf("Right address change failed\r\n");
    } else {
        printf("Right address changed to 0x%02X\r\n", TOF_ADDR_RIGHT);
    }
    
    for (uint32_t i = 0; i < 400000; i++) { __NOP(); } // 稳定延时
    
    // 初始化传感器（使用新地址）
    if (!InitSensor(&tof_left, TOF_ADDR_LEFT)) printf("Left init failed\r\n");
    if (!InitSensor(&tof_right, TOF_ADDR_RIGHT)) printf("Right init failed\r\n");
    
    printf("MultiTOF_Init finished\r\n");
}



// 轮询读取左右传感器距离
void MultiTOF_Update(void)
{
    VL53L1_RangingMeasurementData_t data;
    if (VL53L1_WaitMeasurementDataReady(&tof_left) == VL53L1_ERROR_NONE) {
        VL53L1_GetRangingMeasurementData(&tof_left, &data);
        dist_left = data.RangeMilliMeter;
        VL53L1_ClearInterruptAndStartMeasurement(&tof_left);
    }
    if (VL53L1_WaitMeasurementDataReady(&tof_right) == VL53L1_ERROR_NONE) {
        VL53L1_GetRangingMeasurementData(&tof_right, &data);
        dist_right = data.RangeMilliMeter;
        VL53L1_ClearInterruptAndStartMeasurement(&tof_right);
    }
}

// 马达控制（如果你不需要，可以删除此函数，并在调用处注释）
static uint8_t DistanceToDuty(int32_t dist)
{
    if (dist >= 300) return 20;
    else if (dist <= 100) return 100;
    else return 60;
}

void SetMotorDutyByDistance(int32_t dist, uint8_t channel)
{
    uint8_t duty = DistanceToDuty(dist);
    switch(channel) {
        case 1: __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty); break;
        case 2: __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, duty); break;
        case 3: __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, duty); break;
        default: break;
    }
}

