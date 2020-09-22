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


void _test_pwm(void)
{
  printf("Test PWM...\n");
  TIM_TypeDef *timer = TIM1;
  int channel = 1;
  Pin_t pwm = {GPIOA, 8};
  Pin_t direction = {GPIOB, 12};
  Pin_t brake = {GPIOB, 13};

  // motor timer
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
  
  // encoder timers
  //RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
  //RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
        
  gpio_pin_mode(&pwm, GPIO_MODE_OUTPUT_OPEN_DRAIN | GPIO_MODE_OUTPUT_SPEED_HI | GPIO_MODE_OUTPUT_ALTERNATE);
  gpio_pin_mode(&direction, GPIO_OUTPUT);
  gpio_pin_mode(&brake, GPIO_OUTPUT);

  pwm_start(timer, MOTOR_PWM_PERIOD);
  pwm_pulse_width(timer, channel, MOTOR_PWM_PERIOD / 3);
  pwm_enable_channel(timer, channel);  
  for (;;);
}


void _test_led(void) 
{  
  printf("Test LED...\n");
  for (;;) {
    printf("ON\n");
    led_on();
    delay_ms(500);
    printf("OFF\n");
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
