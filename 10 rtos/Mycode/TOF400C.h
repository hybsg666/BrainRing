#ifndef __TOF400C_H
#define __TOF400C_H

#pragma import(__use_no_semihosting)
#include "TOF400C.h"
#include "vl53l1.h"
#include "main.h"
#include <stdint.h>
#include "stm32h7xx.h"                  // Device header

void TOF400C_Init(void);
void TOF400C_getmotor(void);

#endif 

