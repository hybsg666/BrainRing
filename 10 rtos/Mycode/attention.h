#ifndef __ATTENTION_H
#define __ATTENTION_H

#include <stdint.h>

#define FFT_SIZE 256
#define SAMPLING_FREQ 250
#define MEDIAN_WINDOW 5
#define SMOOTH_WINDOW 5

void attention_init(void);
void attention_set_offset(uint16_t offset);
uint8_t compute_attention_from_adc(const uint16_t *adc_buffer);

#endif
