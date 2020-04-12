#pragma once

#include "board/board.h"
#include "hal/gpio.h"


static Pin_t nLED = {GPIOC, 13}; // Active low


void led_init(void)
{
  gpio_pin_mode(&nLED, GPIO_OUTPUT);
}

void led_toggle(void)
{
    gpio_pin_toggle(&nLED);
}

void led_on(void)
{
    gpio_pin_clr(&nLED);
}

void led_off(void)
{
    gpio_pin_set(&nLED);
}
