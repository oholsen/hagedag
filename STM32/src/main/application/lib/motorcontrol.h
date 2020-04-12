#pragma once

#include "pidcontrol.h"

typedef struct {

    uint32_t lastUpdateTime;
    uint32_t nextUpdateTime;

} Updater_t;


typedef struct {

    TIM_TypeDef *timer; // config
    Pin_t ch1;
    Pin_t ch2;

    // ticks
    uint16_t update;
    int32_t sum;

    float speed;
    float speed_lp;
    
} RotationEncoder_t;


typedef struct {
    // all config

    TIM_TypeDef *timer;
    int channel;


    // TODO: different control signals, ref robobob, IN1/IN2, vs DIR/BRAKE
    Pin_t pwm;
    Pin_t direction;
    Pin_t brake;
    //Pin_t current;

    //float motor_threshold_start;
    //float motor_threshold_stop;

} Motor_t;



typedef struct {

    RotationEncoder_t encoder;
    Motor_t motor;
    PIDControl_t pid;

    //float ticks_per_rev; // config
    float set_speed;
    float set_speed_lp;
    float set_speed_alpha;

    float power;

} MotorControl_t;


void  updater_init(Updater_t *u);
float updater_elapsed(const Updater_t *u);
float updater_update(Updater_t *u);

void encoder_init(RotationEncoder_t *encoder);
void encoder_update(RotationEncoder_t *encoder, float dts);
float encoder_get_speed(RotationEncoder_t *encoder);

void motor_init(Motor_t *motor);
void motor_set_power(Motor_t *motor, float power);

void motor_control_init(MotorControl_t *mc, float speed);
float motor_control_update(MotorControl_t *mc, float dts);
void motor_control_set_speed(MotorControl_t *mc, float speed, float lp_alpha);
float motor_control_get_speed(MotorControl_t *mc);
