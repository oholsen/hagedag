#include "board/board.h"
#include "board/led.h"
#include "hal/systick.h"
#include "hal/gpio.h"
#include "hal/delay.h"
#include "hal/adc.h"
#include "hal/pwm.h"
#include "hal/usart.h"
#include "hal/usart_rx.h"
#include "lib/motorcontrol.h"
#include "lib/pidcontrol.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define M_PI        3.1415

// tick rate -> power
// 12k ticks per wheel rev
#define PID_Kp      1e-4
#define PID_Ti      0.2
#define PID_Td      1e-3
#define PID_alpha   0.1
#define PID_Imax    0.6


#define BATTERY_INTERVAL                20 // seconds


/*
TIM2 for motors, shared.
TIM3 and TIM4 for encoders.

TODO: current sense on analog input, 1.65V / A
2A max per channel, corresponding to 3.3V.

IA on A,3
IB on A,4

*/

#define MOTOR_PWM_TIMER         TIM2
#define MOTOR_PWM_FREQ          100e3
#define MOTOR_PWM_PERIOD        ((uint32_t) (SYSTEM_CLOCK / MOTOR_PWM_FREQ))

#define MAX_TICK_SPEED          5800
#define MAX_POWER_LP            0.5


// static uint32_t time; // seconds since boot
static float time; // seconds since boot
static char buf[128];
static int status;
#define STATUS_MAX_POWER    1


float set_speed_alpha = 5e-2;

// TODO: move to controller or motor
Pin_t sense_A = {GPIOA, 3};
Pin_t sense_B = {GPIOA, 4};


Pin_t usart1_tx = {GPIOA, 9};
Pin_t usart1_rx = {GPIOA, 10};

MotorControl_t motor_A = {
    .encoder = { .timer = TIM3, .ch1 = {GPIOA, 6}, .ch2 = {GPIOA, 7} },
    .motor = { .timer = MOTOR_PWM_TIMER, .channel = 1, .pwm = {GPIOA, 0}, .direction = {GPIOB, 12}, .brake = {GPIOB, 13} }
};

MotorControl_t motor_B = {
    .encoder = { .timer = TIM4, .ch1 = {GPIOB, 6}, .ch2 = {GPIOB, 7} },
    .motor = { .timer = MOTOR_PWM_TIMER, .channel = 2, .pwm = {GPIOA, 1}, .direction = {GPIOB, 14}, .brake = {GPIOB, 15} }
};

#define motor_R motor_A
#define motor_L motor_B
#define sense_R sense_A
#define sense_L sense_B


// Scheduler for motor control PID updates
Updater_t updater;
float cmd_timeout = 0.0f;  // time when command times out, 0 is timeout state
static float power_A, power_B;

Pin_t vbat_ain_pin = {GPIOA, 5};
Pin_t vbat_gnd_pin = {GPIOC, 15}; // Not used
uint32_t v_bat_scaled = 0;

#define V_BAT_ALPHA             5
// Calibration point: Vbat 12.53V Vin 3.046V ADC 3693
#define ADC_TO_V_BATC            (12.53f / 3693.0f)


static inline float battery_voltage(void)
{
    return (ADC_TO_V_BATC * v_bat_scaled) / (1 << V_BAT_ALPHA);
}


void battery_measure(void)
{
    uint16_t adc = adcf1_measure(vbat_ain_pin.bit);
    if (v_bat_scaled == 0) {
        v_bat_scaled = adc << V_BAT_ALPHA;
        return;
    }
    v_bat_scaled += adc - (v_bat_scaled >> V_BAT_ALPHA);
    // printf("BAT %d %d %.3f\n", adc, v_bat_scaled, battery_voltage());
}


void _reset(void) 
{
    status = 0;
    power_A = power_B = 0;
    motor_control_init(&motor_A, 0.0);
    motor_control_init(&motor_B, 0.0);
}


void _stop(void) 
{
    motor_control_set_speed(&motor_L, 0, 1);
    motor_control_set_speed(&motor_R, 0, 1);  
}

// command speeds are in ticks/s
// rotation is anti-clockwise from above
void _set_motor_speeds(float speed_L, float speed_R) {
    motor_control_set_speed(&motor_L, speed_L, set_speed_alpha);
    motor_control_set_speed(&motor_R, speed_R, set_speed_alpha);
} 


bool _check_command_timeout(void)
{
    if (cmd_timeout != 0.0f && time >= cmd_timeout) {
        snprintf(buf, sizeof(buf), "Timeout %.3f %.3f\n", time, cmd_timeout);
        usart_tx_string(USART1, buf);
        _stop();
        cmd_timeout = 0.0f;
        return false;
    }
    return true;
}


void _set_motion(float speed, float rotation, float timeout)
{
    cmd_timeout = timeout;
    float speed_L = speed - rotation;
    float speed_R = speed + rotation;
    if (_check_command_timeout())
    {
        _set_motor_speeds(speed_L, speed_R);
        snprintf(buf, sizeof(buf), "Ack %.3f m %.2f %.2f %.3f\n", time, speed, rotation, cmd_timeout);
        usart_tx_string(USART1, buf);
    }
}


void _handle_command(void)
{
  uint8_t line_buffer[256];
        
  //if (!usart_rx_input_pending(CONTROL_UART))
  if (!usart_rx_eol(CONTROL_UART))
    return;
  
  const char *line = (const char *) usart_rx_read_line(CONTROL_UART, line_buffer, sizeof(line_buffer), 0.5);
  if (line == NULL)
    return;
  
  printf("Control [%s]\n", line);
  snprintf(buf, sizeof(buf), "Control %s", line);
  usart_tx_string(USART1, buf);
        
  char name[16];
  float arg1, arg2, arg3, arg4, arg5;
  int n = sscanf(line, "%s %f %f %f %f %f", name, &arg1, &arg2, &arg3, &arg4, &arg5);
   
  if (n < 1)
    return;
  
  if (strcmp(name, ".") == 0) {
    _stop();
    return;
  }

  if (strcmp(name, "!") == 0) {
    _reset();
    return;
  }

  if (strcmp(name, "m") == 0 && n >= 4) {
    // arg1 is translation speed forwards in cm/s
    // arg2 is rotation speed in deg/s
    // arg3 is time at which the command times out
    _set_motion(arg1, arg2, arg3);
    return;
  }

  if (strcmp(name, "pid") == 0) {
    
    if (n >= 4) {
      float kp = arg1;
      float ti = arg2;
      float td = arg3;
      
      pidcontrol_config(&motor_L.pid, kp, ti, td, PID_alpha, -PID_Imax, +PID_Imax);
      motor_control_init(&motor_L, 0.0);
      pidcontrol_config(&motor_R.pid, kp, ti, td, PID_alpha, -PID_Imax, +PID_Imax);
      motor_control_init(&motor_R, 0.0);    
    }

    snprintf(buf, sizeof(buf), "PID %g %g %g\n", motor_L.pid.Kp, motor_L.pid.Ti, motor_L.pid.Td);
    usart_tx_string(USART1, buf);
    printf(buf);
    return;
  }
}


void _control_from_uart(void)
{

    printf("Control from UART\n");
    
    // motor timer
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // encoder timers
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    
    // NOTE: current sensing should be synched with PWM!!!
    gpio_pin_mode(&sense_A, GPIO_ANALOG);
    gpio_pin_mode(&sense_B, GPIO_ANALOG);
    gpio_pin_mode(&vbat_ain_pin, GPIO_ANALOG);
    // gpio_pin_mode_f1(&vbat_ain_pin, GPIO_MODE_INPUT_ANALOG);
    
    adcf1_start();
    
    pwm_start(MOTOR_PWM_TIMER, MOTOR_PWM_PERIOD);
    
    pidcontrol_config(&motor_A.pid, PID_Kp, PID_Ti, PID_Td, PID_alpha, -PID_Imax, +PID_Imax);
    motor_control_init(&motor_A, 0.0);

    pidcontrol_config(&motor_B.pid, PID_Kp, PID_Ti, PID_Td, PID_alpha, -PID_Imax, +PID_Imax);
    motor_control_init(&motor_B, 0.0);
    
    // _test_power_speed();    

    uint32_t debugInterval = systick_seconds2ticks(1.0);
    uint32_t nextDebugTicks = systick_add(systick_read(), debugInterval);

    float nextBatteryTime = 0; // seconds
       
    updater_init(&updater);
    uint32_t updateInterval = systick_seconds2ticks(0.025);
    uint32_t nextUpdateTicks = systick_add(systick_read(), updateInterval);
    uint32_t updates = 0;
    
    uint32_t time_ticks = systick_read();    
    
    while (1)
    {

        _handle_command();
        
        uint32_t ticks = systick_read();
        if (systick_expired2(ticks, nextUpdateTicks)) {
            time += 0.025;
            nextUpdateTicks = systick_add(ticks, updateInterval);
            if ((status & STATUS_MAX_POWER) == 0) {
              if (fabs(power_A) < MAX_POWER_LP && fabs(power_B) < MAX_POWER_LP) {
                float dts = updater_update(&updater);
                motor_control_update(&motor_L, dts);
                motor_control_update(&motor_R, dts);
                // approx 8s before overload triggers
                power_A += 0.005 * (motor_A.pid.I - power_A);
                power_B += 0.005 * (motor_B.pid.I - power_B);
                updates++;
              }
              else {
                // Must reset motors to get going again
                printf("Power overload - stopping!\n");
                status |= STATUS_MAX_POWER;
                motor_set_power(&motor_A.motor, 0);
                motor_set_power(&motor_B.motor, 0);
              }
            }
        }
        
        ticks = systick_read();
        if (systick_expired2(ticks, time_ticks + SYSTICK_CLOCK)) {
            time_ticks = systick_add(time_ticks, SYSTICK_CLOCK);
            led_toggle();            
            battery_measure();
        }

        ticks = systick_read();
        if (systick_expired2(ticks, nextDebugTicks)) {
            /////////////////////////////////////////////////////
            // Debug
            /////////////////////////////////////////////////////

            nextDebugTicks = systick_add(ticks, debugInterval); // interval, skew when delayed
            //nextDebugTicks = systick_add(nextDebugTicks, debugInterval);  // periodic, can get serious backlog
                    
            printf("%.3f", time);
            //printf(" %d", updates);
            //printf(" %.3f", speed);
            printf(" %3d %3d", motor_A.encoder.sum, motor_B.encoder.sum);

            MotorControl_t *motor = &motor_A;
            float speed = motor->encoder.speed; // No need to low-pass filter
            printf(" %6.0f", speed);
            printf(" | %.3f | %.3f %.3f %.3f", motor->power, motor->pid.P, motor->pid.I, motor->pid.D);
#if 1
            // must be synched with PWM?? Or add LP filter on input...
            uint16_t current_L = adcf1_measure(sense_L.bit);
            uint16_t current_R = adcf1_measure(sense_R.bit);
            printf(" | %4d  %4d", current_L, current_R);
#endif       
            printf("\n");
            updates = 0;

            snprintf(buf, sizeof(buf), "Time %.3f\n", time);
            usart_tx_string(USART1, buf);

            // Distance travelled - in ticks
            snprintf(buf, sizeof(buf), "Ticks %d %d\n", motor_L.encoder.sum, motor_R.encoder.sum);
            usart_tx_string(USART1, buf);
            
            // Speed in revs/sec - could use cm/sec
            snprintf(buf, sizeof(buf), "Speed %.1f %.1f %.1f %.1f\n", motor_L.encoder.speed, motor_R.encoder.speed, motor_L.set_speed_lp, motor_R.set_speed_lp);
            usart_tx_string(USART1, buf);

            // snprintf(buf, sizeof(buf), "Power %6.3f %6.3f   %6.3f %6.3f\n", motor_L.power, motor_R.power, motor_L.pid.I, motor_R.pid.I);
            snprintf(buf, sizeof(buf), "Power %.3f %.3f\n", motor_L.power, motor_R.power);
            usart_tx_string(USART1, buf);

            snprintf(buf, sizeof(buf), "Status %d\n", status);
            usart_tx_string(USART1, buf);
        }

        _check_command_timeout();

        if (time >= nextBatteryTime) {
            nextBatteryTime = time + BATTERY_INTERVAL;
            float vbat = battery_voltage();
            snprintf(buf, sizeof(buf), "Battery %.3f\n", vbat);
            usart_tx_string(USART1, buf);
            printf(buf);
            // printf("BAT %d %.3f\n", v_bat_scaled, vbat);            
        }
        
    }    
}


// called from startup_stm32*.s
void SystemInit(void) {
}


static uint8_t usart_buffer[512];


// #include "tests.c"


int main(void)
{
    // TODO: use startup.s from CubeF1 LL in case want to manipulate interrupt vectors
    // TODO: could use HSE at 8MHz, can also use 72MHz!?

    // Use HSI at 8MHz x 8
    // Switch to 64MHz clock: 8MHz/2 * 16
    FLASH->ACR |= 2 * FLASH_ACR_LATENCY_0;
    RCC->CFGR = RCC_CFGR_PLLMULL16;
    RCC->CR |= RCC_CR_PLLON;
    while ((RCC->CR & RCC_CR_PLLRDY) == 0);
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
    
    systick_start();
    
    // Enable the clock to GPIO ports
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;


    led_init();
    led_on();

#if 0
    _test_led();
    _test_adc();
    _test_vbat();
    _test_motor();
    _test_pwm();
#endif
    
    //RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    usart_start(USART1, 115200);
    gpio_pin_mode(&usart1_tx, GPIO_MODE_OUTPUT_SPEED_HI | GPIO_MODE_OUTPUT_ALTERNATE);
    gpio_pin_mode(&usart1_rx, GPIO_MODE_INPUT | GPIO_MODE_INPUT_FLOATING);
    gpio_pin_pull_up(&usart1_rx);

#if 0
    for (;;) {
      while (!usart_rx_ready(USART1)); //        printf("S %2x ", USART1->SR);
      uint8_t ch = USART1->DR;
      printf("CH %x %c\n", ch, ch);
    }           
#endif
    
    usart_rx_start(USART1, usart_buffer, sizeof(usart_buffer));
    
    _control_from_uart();
   
#if 0
    while (1)
    {
        uint8_t line_buffer[256];
        
        char *line = (char *) usart_rx_read_line(USART1, line_buffer, sizeof(line_buffer), 3.0);

        led_on();
        delay_ms(500);
        
        led_off();
        delay_ms(500);
        
        printf("Received: %s\n", line);
        usart_tx_string(USART1, "Hello from STM!\n");
    }
#endif
}
