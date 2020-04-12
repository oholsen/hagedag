#pragma once

void pwm_start(TIM_TypeDef *timer, uint32_t period);
void pwm_enable_channel(TIM_TypeDef *timer, uint8_t channel);
void pwm_disable_channel(TIM_TypeDef *timer, uint8_t channel);
void pwm_pulse_width(TIM_TypeDef *timer, uint8_t channel, uint32_t pulse_width);
void pwm_duty_cycle(TIM_TypeDef *timer, uint8_t channel, float duty_cycle);
void pwm_stop(TIM_TypeDef *timer);
