#include <assert.h>
#include <stdio.h>
#include "board/board.h"
#include "adc.h"
#include "delay.h"



void adcf1_start(void)
{
    // Not already started, then will start conversion
    assert((ADC1->CR2 & ADC_CR2_ADON) == 0);
    // TODO: could reset ADC1 from RCC
    
    
    // ADC clock enable
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    // PCLK2 DIV2 by default
    //RCC->CFGR2 = RCC_CFGR2_ADC1PRES_DIV8;
        
    // Enable ADC if not already set, otherwise starts new conversion
    ADC1->CR2 |= ADC_CR2_ADON;
    
    // At least two ADC clock cycles
    delay_us(400); // TODO: check

    // Calibrate
    ADC1->CR2 |= ADC_CR2_CAL;
    while ((ADC1->CR2 & ADC_CR2_CAL) != 0);

}

#if 0
    printf("CR %08X  ISR %08X\n", ADC1->CR, ADC1->ISR);
#endif


static inline uint32_t set_bit_group(uint32_t dest, unsigned int bits, unsigned int offset, uint32_t value) {
    uint32_t mask = (2^bits - 1) << offset;
    value = (value << offset) & mask;
    return (dest & ~mask) | value;
}

void adcf1_sample_time(uint32_t channel, uint32_t sample_time_bits) {
  if (channel <= 9)
    ADC1->SMPR2 = set_bit_group(ADC1->SMPR2, 3, 3 * channel, sample_time_bits);
  else if (channel <= 17)
    ADC1->SMPR1 = set_bit_group(ADC1->SMPR1, 3, 3 * (channel - 10), sample_time_bits);
}

uint16_t adcf1_measure(uint32_t channel)
{
#if 0
    printf("ADCF1 measure %d\n", channel);
    printf("CR %08X  ISR %08X\n", ADC1->CR, ADC1->ISR);
#endif

    // TODO: adcfq_sample time(uint32_t channel, VALUE);
    // sample time registers, 3 bits per channel
    assert(channel <= 9); // SMPR1 for higher channels!!
    ADC1->SMPR2 = 5 << (3 * channel); // 55.5 ADC clock cycles 

    ADC1->SQR1 = 0 << 20; // 1 conversion sequence
    // while ((ADC1->CR2 & ADC_CR2_ADON) != 0);

    // channel for first conversion
    ADC1->SQR3 = (channel * ADC_SQR3_SQ1_0);
    
    // Start and wait for end of conversion
    ADC1->SR &= ~ADC_SR_EOC;
    ADC1->CR2 |= ADC_CR2_ADON;
    while ((ADC1->SR & ADC_SR_EOC) == 0);

    uint16_t adc = ADC1->DR;
    return adc;
}

