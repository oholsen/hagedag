#include "board/board.h"
#include "hal/systick.h"
#include "hal/gpio.h"
#include "hal/pwm.h"
#include "motorcontrol.h"
#include <stdio.h>
#include <math.h>


float encoder_speed_alpha = 0.1f;


void updater_init(Updater_t *u) {
    long t = systick_read();
    u->lastUpdateTime = t;
}

float updater_elapsed(const Updater_t *u) {
    long t = systick_read();
    return systick_elapsed(u->lastUpdateTime, t);
}

float updater_update(Updater_t *u) {
    long t = systick_read();
    uint32_t dt = systick_elapsed(u->lastUpdateTime, t);
    u->lastUpdateTime = t;
    return systick_ticks2seconds(dt);
}


static void _encoder_start_timer(TIM_TypeDef *timer)
{
    timer->CNT = 0;
    timer->SMCR = 3 * TIM_SMCR_SMS_0;
    timer->CCMR1 = TIM_CCMR1_CC2S_0 | TIM_CCMR1_CC1S_0;
    timer->EGR = TIM_EGR_UG;
    timer->CR1 = TIM_CR1_CEN;
}

void encoder_init(RotationEncoder_t *encoder)
{
    // ch1 and ch2 are by default input.
    // Should enable pull-down?
    _encoder_start_timer(encoder->timer);
    encoder->speed = 0;
    encoder->speed_lp = 0;
    encoder->update = encoder->timer->CNT;
}

void encoder_update(RotationEncoder_t *encoder, float dts)
{
    uint16_t value = encoder->timer->CNT;
    int16_t delta = value - encoder->update;
    encoder->speed = delta / dts;
    encoder->speed_lp += encoder_speed_alpha * (encoder->speed - encoder->speed_lp); 
    encoder->update = value;
    encoder->sum += delta;
}

float encoder_get_speed(RotationEncoder_t *encoder)
{
    return encoder->speed_lp;
}

void motor_init(Motor_t *motor)
{
    gpio_pin_mode(&motor->pwm, GPIO_MODE_OUTPUT_SPEED_HI | GPIO_MODE_OUTPUT_ALTERNATE);
    gpio_pin_mode(&motor->direction, GPIO_OUTPUT);
    gpio_pin_mode(&motor->brake, GPIO_OUTPUT);
    //gpio_pin_mode(motor->current, GPIO_MODE_ANALOG);

    pwm_pulse_width(motor->timer, motor->channel, 0);
    pwm_enable_channel(motor->timer, motor->channel);
}

void motor_set_power(Motor_t *motor, float power)
{
    float duty = fabs(power);
    if (duty > 1)
        duty = 1;
    if (duty < 0)
        duty = 0;

    //printf("powerA %.3f %.3f\n", power, duty);
    gpio_pin_assign(&motor->direction, power >= 0);
    pwm_duty_cycle(motor->timer, motor->channel, duty);
}


void motor_control_init(MotorControl_t *mc, float speed)
{
    encoder_init(&mc->encoder);
    motor_init(&mc->motor);
    mc->set_speed = speed;
    mc->set_speed_alpha = 0;
    mc->set_speed_lp = speed;
    pidcontrol_init(&mc->pid, speed);
}

void motor_control_set_speed(MotorControl_t *mc, float speed, float lp_alpha)
{
    mc->set_speed_alpha = lp_alpha;
    mc->set_speed = speed;
}

float motor_control_get_speed(MotorControl_t *mc)
{
    return encoder_get_speed(&mc->encoder); //  / mc->ticks_per_rev;
}

float motor_control_update(MotorControl_t *mc, float dts)
{
    encoder_update(&mc->encoder, dts);
    mc->set_speed_lp += mc->set_speed_alpha * (mc->set_speed - mc->set_speed_lp);
    
    // Make sure the power is 0 when the set speed is 0.
    // If the encoder fails, the PID controller saturates power on set speed 0.
    // TODO: look for mishap at a slower scale, integrate "difference" between power and speed and signal error
    float power = 0.0f;
    // if (!mc->pid.saturated && fabs(mc->set_speed_lp) > 0.001) {
    if (fabs(mc->set_speed_lp) > 0.001) {
        float speed = mc->encoder.speed; //motor_control_get_speed(mc);
        power = pidcontrol_update(&mc->pid, mc->set_speed_lp, speed, dts);        
#if 0
        // fail if PID controller saturated, will keep integrator saturated
        if (mc->pid.saturated)
           power = 0.0f;
#endif
    }
    
    motor_set_power(&mc->motor, power);
    mc->power = power;
    return power;
}
