#include "vl53l1.h"

int32_t distance;
VL53L1_Dev_t VL53;
VL53L1_RangingMeasurementData_t result_data;
mode_data Mode_data[]=
{
	{(FixPoint1616_t)(16384), //its uint32_t, uint16_.uint16_t, 0.25*65536
	 (FixPoint1616_t)(1179648),	 //18*65536
	 33000,
	 14,
	 10},//default
		
	{(FixPoint1616_t)(16384),	//0.25*65536
	 (FixPoint1616_t)(1179648),		//18*65536
	 200000, 
	 14,
	 10},//high accuracy
		
  {(FixPoint1616_t)(6554),		//0.1*65536
	 (FixPoint1616_t)(3932160),		//60*65536
	 33000,
	 18,
	 14},//long distance
	
  {(FixPoint1616_t)(16384),	//0.25*65536
	 (FixPoint1616_t)(2097152),		//32*65536
	 20000,
	 14,
	 10},//high speed
};

VL53L1_Error VL53L1Init(VL53L1_Dev_t* pDev)
{
    printf("Enter VL53L1Init\r\n");
    // 1. 将 XSHUT (PB9) 拉低，保持至少 10ms，确保芯片彻底复位
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
    HAL_Delay(20);
    // 2. 拉高 XSHUT，启动芯片
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
    HAL_Delay(50);
    
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    pDev->I2cDevAddr = 0x29; // 7位地址
    pDev->comms_type = 1;
    pDev->comms_speed_khz = 400;
    
  //  printf("Before WaitDeviceBooted\r\n");
    Status = VL53L1_WaitDeviceBooted(pDev);
   // printf("After WaitDeviceBooted, Status=%d\r\n", Status);
    if(Status != VL53L1_ERROR_NONE)
    {
      //  printf("Wait device Boot failed!\r\n");
        return Status;
    }
    HAL_Delay(2);
    
   // printf("Before DataInit\r\n");
    Status = VL53L1_DataInit(pDev);
   // printf("After DataInit, Status=%d\r\n", Status);
    if(Status != VL53L1_ERROR_NONE) 
    {
      //  printf("datainit failed!\r\n");
        return Status;
    }
    HAL_Delay(2);
    
   // printf("Before StaticInit\r\n");
    Status = VL53L1_StaticInit(pDev);
   // printf("After StaticInit, Status=%d\r\n", Status);
    if(Status != VL53L1_ERROR_NONE) 
    {
       // printf("static init failed!\r\n");
        return Status;
    }
    HAL_Delay(2);
    
    //printf("Before SetDistanceMode\r\n");
    Status = VL53L1_SetDistanceMode(pDev, VL53L1_DISTANCEMODE_LONG);
   // printf("After SetDistanceMode, Status=%d\r\n", Status);
    if(Status != VL53L1_ERROR_NONE) 
    {
       // printf("set distance mode failed!\r\n");
        return Status;
    }
    HAL_Delay(2);
    
    //printf("VL53L1Init completed successfully\r\n");
    return Status;
}

/** 
* @brief  传感器进行一个出厂校准
* @param [in] pDev  指定传感器
* @param [in] save  结果存储地址
* @retval  VL53L1_Error类型 
* @par 日志 
*
*/
VL53L1_Error VL53Cali(VL53L1_Dev_t* pDev,void * save)
{
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    Status = VL53L1_StopMeasurement(pDev);
	if(Status!=VL53L1_ERROR_NONE) 
		return Status;
    Status = VL53L1_PerformRefSpadManagement(pDev);//perform ref SPAD management
	if(Status!=VL53L1_ERROR_NONE) 
		return Status;
    /*
    Status = VL53L1_PerformOffsetSimpleCalibration(pDev,140);//14cm的出厂校验值
	if(Status!=VL53L1_ERROR_NONE) 
		return Status;
    */
    Status = VL53L1_GetCalibrationData(pDev,save);
	if(Status!=VL53L1_ERROR_NONE) 
		return Status;
    //全部完成 重新打开测量
    Status = VL53L1_StartMeasurement(pDev);
    return Status;
}

VL53L1_Error VL53InitParam(VL53L1_Dev_t* pDev, uint8_t mode)
{
    //printf("VL53InitParam: mode=%d\r\n", mode);
    VL53L1_Error status = VL53L1_ERROR_NONE;
    
   // printf("  SetLimitCheckEnable SIGMA...\r\n");
    status = VL53L1_SetLimitCheckEnable(pDev, VL53L1_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
    if(status != VL53L1_ERROR_NONE) {
    //    printf("  failed, status=%d\r\n", status);
        return status;
    }
    HAL_Delay(2);
    
   // printf("  SetLimitCheckEnable SIGNAL_RATE...\r\n");
    status = VL53L1_SetLimitCheckEnable(pDev, VL53L1_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    if(status != VL53L1_ERROR_NONE) {
   //     printf("  failed, status=%d\r\n", status);
        return status;
    }
    HAL_Delay(2);
    
   // printf("  SetLimitCheckValue SIGMA...\r\n");
    status = VL53L1_SetLimitCheckValue(pDev, VL53L1_CHECKENABLE_SIGMA_FINAL_RANGE, Mode_data[mode].sigmaLimit);
    if(status != VL53L1_ERROR_NONE) {
    //    printf("  failed, status=%d\r\n", status);
        return status;
    }
    HAL_Delay(2);
    
    //printf("  SetLimitCheckValue SIGNAL_RATE...\r\n");
    status = VL53L1_SetLimitCheckValue(pDev, VL53L1_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, Mode_data[mode].signalLimit);
    if(status != VL53L1_ERROR_NONE) {
    //    printf("  failed, status=%d\r\n", status);
        return status;
    }
    
   // printf("  SetMeasurementTimingBudget...\r\n");
    status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(pDev, Mode_data[mode].timingBudget);
    if(status != VL53L1_ERROR_NONE) {
    //    printf("  failed, status=%d\r\n", status);
        return status;
    }
    HAL_Delay(2);
    
   // printf("  SetInterMeasurementPeriod...\r\n");
    status = VL53L1_SetInterMeasurementPeriodMilliSeconds(pDev, 300);
    if(status != VL53L1_ERROR_NONE) {
    //    printf("  SetInterMeasurementPeriodMilliSeconds failed! status=%d\r\n", status);
        return status;
    }
    HAL_Delay(2);
    
   // printf("  StartMeasurement...\r\n");
    status = VL53L1_StartMeasurement(pDev);
    if(status != VL53L1_ERROR_NONE) {
     //   printf("  start measurement failed! status=%d\r\n", status);
        return status;
    }
  //  printf("VL53InitParam completed successfully\r\n");
    return status;
}
int32_t flag=0;
VL53L1_Error getDistance(VL53L1_Dev_t* pDev)
{
	//    printf("getDistance enter\r\n");
    fflush(stdout);  // 确保立即输出
	
	VL53L1_Error status = VL53L1_ERROR_NONE;
    uint8_t isDataReady=1;
    status = VL53L1_WaitMeasurementDataReady(pDev);
    //status = VL53L1_GetMeasurementDataReady(pDev,&isDataReady);
    if(status!=VL53L1_ERROR_NONE) 
	{
	//	printf("WaitMeasurementDataReady error code: %d\r\n",status);
		return status;
	}
    if(!isDataReady)
    {
        flag++;
        return -7;
    }
    status = VL53L1_GetRangingMeasurementData(pDev, &result_data);
    distance = result_data.RangeMilliMeter;
    status = VL53L1_ClearInterruptAndStartMeasurement(pDev);
    return status;
}
