/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "servo.h"          
#include <stdio.h>
#include "queue.h" 
#include "attention.h"
#include "mpu6050.h"      
#include "inv_mpu.h"      
#include "inv_mpu_dmp_motion_driver.h"  
#include "3TOF3Motor.h"
#include "balance.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SIM_ATT_MIN     0
#define SIM_ATT_MAX     100
#define SIM_STEP        1
#define SIM_DELAY_MS    50      // 模拟专注度变化间隔 (ms)

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart3;

typedef struct {
    float amplitude;        // 摆动幅度（度）
    uint16_t half_period_ms; // 摆动半周期（ms）
} SwingParams_t;

static const SwingParams_t swing_modes[4] = {
    // 索引0不用，档位1~3对应1,2,3
    {0.0f, 0},          // 占位
    {30.0f, 800},       // 档位1: 极度不专注 -> 大幅摆动，周期1.6秒
    {15.0f,  600},       // 档位2: 稍微不专注 -> 中幅摆动，周期1.2秒
    {0.0f,  0}        // 档位3: 专注 -> 轻微摆动（几乎静止）
};

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

float current_roll = 0.0f;

// 队列句柄：用于传递从 L431 收到的完整数据行（每行最大32字节）
QueueHandle_t xUART_LineQueue = NULL;
// 队列句柄：用于传递专注度值 (0~100)
QueueHandle_t xAttentionQueue = NULL;
 // 队列句柄：用于传递距离
QueueHandle_t xTOFQueue = NULL;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void vAttentionSimulatorTask(void *pvParameters);
void vServoControlTask(void *pvParameters);
void vUART_ProcessTask(void *pvParameters);
void vMPU6050_Task(void *pvParameters);
void vTOF_ReadTask(void *pvParameters);
void vBalanceControlTask(void *pvParameters);
//void vMPU6050_Task(void *pvParameters);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
	
  // 创建专注度队列（容量10，每个元素为 uint8_t）
  xAttentionQueue = xQueueCreate(50, sizeof(uint8_t));
  if (xAttentionQueue == NULL) while(1);
  
  // 创建行数据队列（容量20，每个元素为32字节，足够存放一行 "ch1,ch2\n" 字符串
  xUART_LineQueue = xQueueCreate(20, 32); 
  if (xUART_LineQueue == NULL) while(1);
  
  // 创建距离队列（容量10，每个元素为 uint32_t）
  xTOFQueue = xQueueCreate(10, sizeof(uint32_t)*2);
  if (xTOFQueue == NULL) while(1);   // 修正变量名
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  
  
  // 创建 UART 数据处理任务（处理真实脑电数据，优先级与模拟任务相同）
  xTaskCreate(vUART_ProcessTask, "UARTProc", 512, NULL, 2, NULL);
//  // 创建模拟专注度生成任务（优先级较高）
  //xTaskCreate(vAttentionSimulatorTask, "AttentionSim", 256, NULL, 2, NULL);
  // 创建舵机控制任务（优先级较低）
  //xTaskCreate(vServoControlTask, "ServoCtrl", 256, NULL, 1, NULL);
  // 创建陀螺仪读数任务（优先级较低）
  xTaskCreate(vMPU6050_Task, "MPU6050", 512, NULL, 1, NULL);
  // 创建读取距离任务（优先级较高）
  xTaskCreate(vTOF_ReadTask, "TOFRead", 512, NULL, 2, NULL);
  // 创建平衡台控制任务（堆栈稍大）
  xTaskCreate(vBalanceControlTask, "BalanceCtrl", 1024, NULL, 2, NULL);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

// 模拟专注度生成任务（三角波，0~100 循环）
void vAttentionSimulatorTask(void *pvParameters)
{
    uint8_t sim_attention = 50;
    int8_t step = SIM_STEP;
    for (;;)
    {
        sim_attention += step;
        if (sim_attention >= SIM_ATT_MAX) { sim_attention = SIM_ATT_MAX; step = -SIM_STEP; }
        else if (sim_attention <= SIM_ATT_MIN) { sim_attention = SIM_ATT_MIN; step = SIM_STEP; }

        // 直接发送 0~100 的值
        xQueueSend(xAttentionQueue, &sim_attention, portMAX_DELAY);

        vTaskDelay(pdMS_TO_TICKS(SIM_DELAY_MS));
    }
}


// 舵机控制任务：接收专注度值，平滑转动到对应角度
void vServoControlTask(void *pvParameters)
{
    uint8_t current_level = 2;   // 初始档位
    float target_angle = 0.0f;
    int direction = 1;            // 1: 向右倾斜, -1: 向左倾斜
    uint32_t last_change = 0;
    
    last_change = xTaskGetTickCount();
    
    for (;;)
    {
        uint8_t new_level;
        if (xQueueReceive(xAttentionQueue, &new_level, 0) == pdTRUE)
        {
            if (new_level >= 1 && new_level <= 3 && new_level != current_level)
            {
                current_level = new_level;
                printf("[ATT] Swing mode changed to %d\r\n", current_level);
            }
        }
        
        SwingParams_t params = swing_modes[current_level];
        uint32_t now = xTaskGetTickCount();
        if (params.half_period_ms > 0 && (now - last_change) >= pdMS_TO_TICKS(params.half_period_ms))
        {
            direction = -direction;
            target_angle = direction * params.amplitude;
            last_change = now;
            int16_t servo_angle = 90 + (int16_t)target_angle;
            if (servo_angle < 0) servo_angle = 0;
            if (servo_angle > 180) servo_angle = 180;
            Servo_SmoothToAngle((uint8_t)servo_angle, 5);
        }
        
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}


// UART 数据处理任务：从队列接收行数据，解析 ch1,ch2，攒够 256 对后计算专注度并发送到专注度队列
void vUART_ProcessTask(void *pvParameters)
{
    char line[32];
    uint16_t ch1_buf[256], ch2_buf[256];
    uint16_t sample_cnt = 0;

    for (;;)
    {
        if (xQueueReceive(xUART_LineQueue, &line, portMAX_DELAY) == pdTRUE)
        {
			 //printf("Received line: %s\n", line); 
            uint16_t ch1, ch2;
            if (sscanf(line, "%hu,%hu", &ch1, &ch2) == 2)
            {
                ch1_buf[sample_cnt] = ch1;
                ch2_buf[sample_cnt] = ch2;
                sample_cnt++;
                if (sample_cnt == 256)
                {
                    // 计算专注度
					uint8_t attention = compute_attention_from_adc(ch1_buf);
					
					// 打印到串口（USART1）
					//printf("Attention: %d\r\n", attention);
					
					
					// 发送到手机 APP（USART3）
					char msg[8];
					sprintf(msg, "%d\n", attention);
					HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);
					
					// 发送到舵机控制队列
					xQueueSend(xAttentionQueue, &attention, 0);
					
					sample_cnt = 0;
                }
            }
        }
    }
}


// MPU6050 数据读取任务（使用 DMP 获取姿态角）
void vMPU6050_Task(void *pvParameters)
{
    float pitch = 0, roll = 0, yaw = 0;
    for (;;)
    {
        if (mpu_dmp_get_data(&pitch, &roll, &yaw) == 0) {
            current_roll = roll;   // 更新全局 Roll 角
            // 可选打印，可不打印
            // printf("[MPU]Pitch: %.2f, Roll: %.2f, Yaw: %.2f\r\n", pitch, roll, yaw);
        }
        vTaskDelay(pdMS_TO_TICKS(100));  // 10Hz 更新频率足够
    }
}
// 距离读取任务
void vTOF_ReadTask(void *pvParameters)
{
    MultiTOF_Init();
    for (;;)
    {
        MultiTOF_Update();
        printf("Left: %ld mm, Right: %ld mm\r\n", dist_left, dist_right);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}


// 平衡台控制任务（放在 freertos.c 中）
void vBalanceControlTask(void *pvParameters)
{
    uint8_t attention = 50;
    for (;;)
    {
        // 获取专注度（非阻塞）
        xQueueReceive(xAttentionQueue, &attention, 0);
        
        // 直接读取全局距离（在 vTOF_ReadTask 中更新）
        extern int32_t dist_left, dist_right;
        Balance_Update(attention, (float)dist_left, (float)dist_right);
        

		
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}




/* USER CODE END Application */

