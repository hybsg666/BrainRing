// balance.h
#ifndef __BALANCE_H
#define __BALANCE_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

void Balance_Init(void);
void Balance_Update(uint8_t attention, float left_dist, float right_dist);
uint16_t Balance_GetTargetAngle(void);

#endif
