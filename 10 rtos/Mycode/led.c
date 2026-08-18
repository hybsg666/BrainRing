#include "stm32h7xx.h"                  // Device header

#include "led.h"  


void LED_Init(void)
{

	HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_RESET);		// LED1Òý½ÅÊä³öµÍ£¬¼´µãÁÁLED1
	
	
}
