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

#if 1
#define PID_Ti      0.2
#define PID_Td      1e-3
#define PID_alpha   0.1
#define PID_Imax    1.0
#else
#define PID_Ti      0
#define PID_Td      0
#define PID_alpha   0
#define PID_Imax    0
#endif


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

#define TICKS_PER_REV           (270.0 * 36) // gear ratio * hall ticks/rev
#define WHEEL_DIAM              12.5 // cm
#define WHEEL_BASE              33.0 // cm
#define DIST_PER_REV            (M_PI * WHEEL_DIAM) // cm
#define DIST_TO_REV             (1 / DIST_PER_REV)
#define ROTATION_DIST           (M_PI * WHEEL_BASE) // cm
#define TICKS_PER_CM            (TICKS_PER_REV / DIST_PER_REV)
#define MAX_TICK_SPEED          6800


static char buf[128];
// static int status;


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
float cmd_speed = 0.0f;    // forward revs / sec
float cmd_rotation = 0.0f; // right from above
float cmd_timeout = 0.0f;  // time when command times out, 0 is timeout state


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


void _test_adc(void)
{
    Pin_t pin = {GPIOA, 2};
    printf("Test ADC\n");
    gpio_pin_mode(&pin, GPIO_MODE_INPUT_ANALOG);
    adcf1_start();
    for (;;) {
        delay_ms(500);
        uint16_t v = adcf1_measure(pin.bit);
        printf("%4d\n", v);
    }
}


/*
On 10k? connected to 3V3, grounded via C15:
3.24V high ADC 3994
23mV low ADC 15
+-3 ADCs on both readings, add LP average.

Not a good design. Ain was 4V when not grounded (over-voltage protection from 12-14V).

*/
void _test_vbat(void)
{
    printf("Test Vbat\n");
    gpio_pin_mode(&vbat_ain_pin, GPIO_MODE_INPUT_ANALOG);
    gpio_pin_mode(&vbat_gnd_pin, GPIO_MODE_OUTPUT_OPEN_DRAIN | GPIO_MODE_OUTPUT_SPEED_LOW); 
    adcf1_start();
    for (;;) {
        uint16_t v;

        printf("1 ");
        gpio_pin_set(&vbat_gnd_pin);
        delay_ms(1000);
        v = adcf1_measure(vbat_ain_pin.bit);
        printf("%4d\n", v);
        delay_ms(1000);
        
        printf("0 ");
        gpio_pin_clr(&vbat_gnd_pin);
        delay_ms(1000);
        v = adcf1_measure(vbat_ain_pin.bit);
        printf("%4d\n", v);
        delay_ms(1000);
    }
  
  
}


void _reset(void) 
{
    // status = 0;
    motor_control_init(&motor_A, 0.0);
    motor_control_init(&motor_B, 0.0);
}


void _stop(void) 
{
    cmd_speed = cmd_rotation = 0;
    motor_control_set_speed(&motor_L, 0, 1);
    motor_control_set_speed(&motor_R, 0, 1);  
}


// command speeds are in cm/s, rotation is to the right from above
void _set_motor_speeds(void) {
    float speed_L = (cmd_speed + cmd_rotation) * TICKS_PER_CM;
    float speed_R = (cmd_speed - cmd_rotation) * TICKS_PER_CM;
    // Guard against overspeed
    /*
    if (fabs(speed_L) > MAX_TICK_SPEED || fabs(speed_R) > MAX_TICK_SPEED) {
      status = 1; // over speed
      _stop();
      return;
    }
    */
    motor_control_set_speed(&motor_L, speed_L, set_speed_alpha);
    motor_control_set_speed(&motor_R, speed_R, set_speed_alpha);
} 


bool _check_command_timeout()
{
    if (cmd_timeout != 0.0f && time >= cmd_timeout) {
        snprintf(buf, sizeof(buf), "Timeout %.3f m %.2f %.2f %.3f\n", time, cmd_speed, cmd_rotation, cmd_timeout);
        usart_tx_string(USART1, buf);
        _stop();
        cmd_timeout = 0.0f;
        return false;
    }
    return true;
}


void _set_motion(float speed, float rotation, float timeout)
{
    cmd_speed = speed;
    cmd_rotation = rotation * (ROTATION_DIST / 360.0); // deg/s -> cm/s
    cmd_timeout = timeout;
    if (_check_command_timeout())
    {
        _set_motor_speeds();
        snprintf(buf, sizeof(buf), "Ack %.3f m %.2f %.2f %.3f\n", time, cmd_speed, cmd_rotation, cmd_timeout);
        usart_tx_string(USART1, buf);
    }
}


void _handle_command(uint32_t time)
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

void _test_led(void) 
{  
  for (;;) {
    led_on();
    delay_ms(500);
    led_off();
    delay_ms(500);      
  }
}



/*
Max speed under no load with 12V Pb battery is 7000 ticks per second.
Nearly there already at 30-40% power. So should guard command input against
over speed, or the integrator will saturate.
*/
void _test_power_speed(void)
{
  MotorControl_t *motor_control = &motor_R;
  
  
  for (float power = 0; power <= 1.0f; power += 0.1f) { 
    motor_set_power(&motor_control->motor, power);

    updater_init(&updater);
    uint32_t updateInterval = systick_seconds2ticks(0.025);
    uint32_t nextUpdateTicks = systick_add(systick_read(), updateInterval);
    uint32_t updates = 0;
    
    while (updates < 400) {
        uint32_t ticks = systick_read();
        if (systick_expired2(ticks, nextUpdateTicks)) {
            nextUpdateTicks = systick_add(ticks, updateInterval);
            float dts = updater_update(&updater);
            encoder_update(&motor_control->encoder, dts);
            updates++;
            if (updates % 40 == 0)
              printf("SPEED %g %g %g\n", power, motor_control->encoder.speed, motor_control->encoder.speed_lp);
        }        
    }
  }
  
  motor_set_power(&motor_control->motor, 0.0);
  for (;;);
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

    uint32_t nextBatteryTime = 0; // seconds
       
    updater_init(&updater);
    uint32_t updateInterval = systick_seconds2ticks(0.025);
    uint32_t nextUpdateTicks = systick_add(systick_read(), updateInterval);
    uint32_t updates = 0;
    
    uint32_t time = 0;
    uint32_t time_ticks = systick_read();    
    
    while (1)
    {

        _handle_command(time);
        
        uint32_t ticks = systick_read();
        if (systick_expired2(ticks, nextUpdateTicks)) {
            nextUpdateTicks = systick_add(ticks, updateInterval);
            float dts = updater_update(&updater);
            if ((motor_L.pid.saturated != 0) || (motor_R.pid.saturated != 0)) {
              // set point only, stop motors in motor_control_update()
              // TODO: add some timeout, eg 10 seconds, for overload...
              // _stop();
            }
            motor_control_update(&motor_L, dts);
            motor_control_update(&motor_R, dts);
            updates++;
        }
        
        ticks = systick_read();
        if (systick_expired2(ticks, time_ticks + SYSTICK_CLOCK)) {
            time++;
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
                    
            printf("%3d", time);
            //printf(" %d", updates);
            //printf(" %.3f", speed);
            printf(" %3d %3d", motor_A.encoder.sum, motor_B.encoder.sum);

            MotorControl_t *motor = &motor_A;
            float speed = motor->encoder.speed; // No need to low-pass filter
            printf(" %6.0f %6.3f", speed, speed / TICKS_PER_REV);
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
            snprintf(buf, sizeof(buf), "Revs %.2f %.2f\n", motor_L.encoder.sum / TICKS_PER_REV, motor_R.encoder.sum / TICKS_PER_REV);
            usart_tx_string(USART1, buf);
            
            // Speed in revs/sec - could use cm/sec
            snprintf(buf, sizeof(buf), "Speed %.3f %.3f %.3f %.3f\n",
                motor_L.encoder.speed / TICKS_PER_REV, motor_R.encoder.speed / TICKS_PER_REV,
                motor_L.set_speed_lp / TICKS_PER_REV, motor_R.set_speed_lp / TICKS_PER_REV);
            usart_tx_string(USART1, buf);

            // snprintf(buf, sizeof(buf), "Power %6.3f %6.3f   %6.3f %6.3f\n", motor_L.power, motor_R.power, motor_L.pid.I, motor_R.pid.I);
            snprintf(buf, sizeof(buf), "Power %6.3f %6.3f\n", motor_L.power, motor_R.power);
            usart_tx_string(USART1, buf);

#if 0
            snprintf(buf, sizeof(buf), "Status %d %d %d\n", status, motor_L.pid.saturated, motor_R.pid.saturated);
            usart_tx_string(USART1, buf);
#endif
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
