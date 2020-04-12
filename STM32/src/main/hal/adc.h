#pragma once

// Guards against redefine if including multiple adc*.h
#define ADC_TYPE       ADCF1


void adcf1_start(void);
uint16_t adcf1_measure(uint32_t channel);
