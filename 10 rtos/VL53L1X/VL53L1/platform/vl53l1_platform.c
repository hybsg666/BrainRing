#include "vl53l1_platform.h"
#include "vl53l1_api.h"
#include "main.h"
#include <string.h>

extern I2C_HandleTypeDef hi2c1;

VL53L1_Error VL53L1_WriteMulti(VL53L1_DEV Dev, uint16_t index, uint8_t *pdata, uint32_t count)
{
	//printf("WriteMulti addr=0x%02X reg=0x%04X len=%d\r\n", Dev->I2cDevAddr, index, count);
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c1, (Dev->I2cDevAddr << 1), index,
                                                  I2C_MEMADD_SIZE_16BIT, pdata, count, 1000);
    //printf("HAL_I2C_Mem_Write status=%d\r\n", status);
    return (status == HAL_OK) ? VL53L1_ERROR_NONE : VL53L1_ERROR_CONTROL_INTERFACE;
}

VL53L1_Error VL53L1_ReadMulti(VL53L1_DEV Dev, uint16_t index, uint8_t *pdata, uint32_t count)
{
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, (Dev->I2cDevAddr << 1), index,
                                                 I2C_MEMADD_SIZE_16BIT, pdata, count, 1000);
    return (status == HAL_OK) ? VL53L1_ERROR_NONE : VL53L1_ERROR_CONTROL_INTERFACE;
}

VL53L1_Error VL53L1_WrByte(VL53L1_DEV Dev, uint16_t index, uint8_t data)
{
    return VL53L1_WriteMulti(Dev, index, &data, 1);
}

VL53L1_Error VL53L1_RdByte(VL53L1_DEV Dev, uint16_t index, uint8_t *data)
{
    return VL53L1_ReadMulti(Dev, index, data, 1);
}

VL53L1_Error VL53L1_WrWord(VL53L1_DEV Dev, uint16_t index, uint16_t data)
{
    uint8_t buf[2];
    buf[0] = (data >> 8) & 0xFF;
    buf[1] = data & 0xFF;
    return VL53L1_WriteMulti(Dev, index, buf, 2);
}

VL53L1_Error VL53L1_RdWord(VL53L1_DEV Dev, uint16_t index, uint16_t *data)
{
    uint8_t buf[2];
    VL53L1_Error status = VL53L1_ReadMulti(Dev, index, buf, 2);
    if (status == VL53L1_ERROR_NONE) {
        *data = (buf[0] << 8) | buf[1];
    }
    return status;
}

VL53L1_Error VL53L1_WrDWord(VL53L1_DEV Dev, uint16_t index, uint32_t data)
{
    uint8_t buf[4];
    buf[0] = (data >> 24) & 0xFF;
    buf[1] = (data >> 16) & 0xFF;
    buf[2] = (data >> 8) & 0xFF;
    buf[3] = data & 0xFF;
    return VL53L1_WriteMulti(Dev, index, buf, 4);
}

VL53L1_Error VL53L1_RdDWord(VL53L1_DEV Dev, uint16_t index, uint32_t *data)
{
    uint8_t buf[4];
    VL53L1_Error status = VL53L1_ReadMulti(Dev, index, buf, 4);
    if (status == VL53L1_ERROR_NONE) {
        *data = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];
    }
    return status;
}

VL53L1_Error VL53L1_UpdateByte(VL53L1_DEV Dev, uint16_t index, uint8_t AndData, uint8_t OrData)
{
    uint8_t data;
    VL53L1_Error status = VL53L1_RdByte(Dev, index, &data);
    if (status == VL53L1_ERROR_NONE) {
        data = (data & AndData) | OrData;
        status = VL53L1_WrByte(Dev, index, data);
    }
    return status;
}

VL53L1_Error VL53L1_GetTickCount(uint32_t *ptick_count_ms)
{
    *ptick_count_ms = HAL_GetTick();
    return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_GetTimerFrequency(int32_t *ptimer_freq_hz)
{
    *ptimer_freq_hz = 1000000;
    return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_WaitMs(VL53L1_DEV Dev, int32_t wait_ms)
{
    HAL_Delay(wait_ms);
    return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_WaitUs(VL53L1_DEV Dev, int32_t wait_us)
{
    volatile uint32_t count = wait_us * (SystemCoreClock / 1000000 / 5);
    while (count--);
    return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_WaitValueMaskEx(VL53L1_DEV Dev, uint32_t timeout_ms,
                                    uint16_t index, uint8_t value, uint8_t mask,
                                    uint32_t poll_delay_ms)
{
    VL53L1_Error status = VL53L1_ERROR_NONE;
    uint32_t start_ms = HAL_GetTick();
    uint8_t byte_value = 0;
    uint8_t found = 0;

    while ((status == VL53L1_ERROR_NONE) &&
           ((HAL_GetTick() - start_ms) < timeout_ms) &&
           (found == 0))
    {
        status = VL53L1_RdByte(Dev, index, &byte_value);
        if ((byte_value & mask) == value)
            found = 1;
        if (status == VL53L1_ERROR_NONE && found == 0 && poll_delay_ms > 0)
            status = VL53L1_WaitMs(Dev, poll_delay_ms);
    }
    if (found == 0 && status == VL53L1_ERROR_NONE)
        status = VL53L1_ERROR_TIME_OUT;
    return status;
}


