#ifndef __ADC_POLL_H
#define __ADC_POLL_H

#include <stdint.h>
#include "adc.h"

void adc_poll_init(void);
void adc_poll_collect(uint16_t *buffer, uint32_t len);

#endif

