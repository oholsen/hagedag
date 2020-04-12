#include <assert.h>
#include "board/board.h"
#include "pwm.h"

void pwm_start(TIM_TypeDef *timer, uint32_t period) {
    timer->ARR  = period - 1; 
    //timer->CR1 |= (TIM_CR1_ARPE | TIM_CR1_CEN);
    timer->CR1 = TIM_CR1_CEN;
    //timer->EGR = TIM_EGR_UG; // TIM1???
}

void pwm_enable_channel(TIM_TypeDef *timer, uint8_t channel) {
    assert(channel >= 1);
    assert(channel <= 4);
    timer->BDTR = TIM_BDTR_MOE; // only for TIM1, in case break detection...
    timer->CCER |= (TIM_CCER_CC1E << (4 * (channel - 1))); // PWM Mode 1    
    // Use CC1NE for using the N pin!
    // No CC4NE????
    
    // 1: On - OC1 signal is output on the corresponding output pin depending on MOE, OSSI, OSSR, OIS1, OIS1N and CC1NE bits.
    
    uint8_t bitofs = 8 * ((channel - 1) % 2); 
    assert((bitofs == 0) || (bitofs == 8));
    //  | TIM_CCMR1_OC1PE  // preload CCRx value
    uint32_t value = (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1) << bitofs;
    //uint32_t value = 0x60 << bitofs;
    // two channels (16 bits) per register on F1, 32 bits, split across low and high 16 bits on F3
    // 8 bits per channel
    
    // CCMR registers are not adjacent on F1
    if (channel <= 2)
        timer->CCMR1 |= value;
    else
        timer->CCMR2 |= value; 
}

void pwm_pulse_width(TIM_TypeDef *timer, uint8_t channel, uint32_t pulse_width) {
    assert(channel >= 1);
    assert(channel <= 4);
    switch (channel) {
    case 1:
        timer->CCR1 = pulse_width;
        break;
    case 2:
        timer->CCR2 = pulse_width;
        break;
    case 3:
        timer->CCR3 = pulse_width;
        break;
    case 4:
        timer->CCR4 = pulse_width;
        break;
    }
    // *(&(timer->CCR1) + channel - 1) = pulse_width; 
}

void pwm_disable_channel(TIM_TypeDef *timer, uint8_t channel) {
    assert(channel >= 1);
    assert(channel <= 4);
    timer->CCER &= ~(TIM_CCER_CC1E << (4 * (channel - 1))) ; 
}

void pwm_stop(TIM_TypeDef *timer) {
    timer->CR1 = 0;
}

void pwm_duty_cycle(TIM_TypeDef *timer, uint8_t channel, float duty_cycle) {
    int32_t pw = (int32_t) (duty_cycle * (timer->ARR - 1));
    if (pw >= timer->ARR)
        pw = timer->ARR - 1;
    if (pw < 0)
        pw = 0;
    pwm_pulse_width(timer, channel, pw);
}