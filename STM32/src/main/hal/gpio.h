#pragma once

#include <assert.h>

typedef struct {
    GPIO_TypeDef *port;
    unsigned int bit;
} Pin_t;



// 4 bits 2xCNF + 2xMODE per pin
// Reset state 0100, i.e. input floating
/*
Mode
0       Input
1       Output, med speed, max 10MHz
2       Output, low speed, max 2MHz
3       Output, hi speed, max 50MHz

CNF (Input)
0       Analog
1       Floating input (reset state)
2       Input with pull up/down
3       Reserved

// option to disable pull? pull is weak?
PxODR for CNF=2:
0       Pull-down
1       Pull-up

CNF (Output)
CNF1    
0       Push-pull
1       Open-drain
CNF0
0       General purpose output
1       Alternate function output

*/


// Need one of these followed by INPUT/OUTPUT specific settings
#define GPIO_MODE_INPUT              0
#define GPIO_MODE_OUTPUT_SPEED_MED   1
#define GPIO_MODE_OUTPUT_SPEED_LOW   2
#define GPIO_MODE_OUTPUT_SPEED_HI    3


// Direct mode to gpio_mode()
#define GPIO_MODE_INPUT_ANALOG               (0 << 2)
#define GPIO_MODE_INPUT_FLOATING             (1 << 2)
#define GPIO_MODE_INPUT_PULL_UP_DOWN         (2 << 2)

#define GPIO_MODE_OUTPUT_PUSH_PULL           (0 << 2)
#define GPIO_MODE_OUTPUT_OPEN_DRAIN          (1 << 2)
#define GPIO_MODE_OUTPUT_GENERAL             (0 << 3)
#define GPIO_MODE_OUTPUT_ALTERNATE           (1 << 3)

// Generic digital output
#define GPIO_OUTPUT	(GPIO_MODE_OUTPUT_PUSH_PULL | GPIO_MODE_OUTPUT_GENERAL | GPIO_MODE_OUTPUT_SPEED_HI)
#define GPIO_ANALOG     (GPIO_MODE_INPUT | GPIO_MODE_INPUT_ANALOG)



static inline void _gpio_pin_mode(GPIO_TypeDef *port, uint32_t pin, uint32_t mode) {
    assert(pin < 16);
    assert(mode < 16);
    __IO uint32_t *reg = &port->CRL;
    if (pin >= 8) {
        reg = &port->CRH;
        pin -= 8;
    }
    assert(pin < 8);
    uint32_t mask = 0xf << (4 * pin);
    *reg = ((*reg) & (~mask)) | (mode << (4 * pin));
}


static inline void gpio_pin_mode(Pin_t *pin, uint32_t mode) {
    _gpio_pin_mode(pin->port, pin->bit, mode);
}

static inline void gpio_pin_set(Pin_t *pin) {
    pin->port->BSRR = 1 << pin->bit;
}

static inline void gpio_pin_clr(Pin_t *pin) {
    pin->port->BRR = 1 << pin->bit;
}

static inline void gpio_pin_toggle(Pin_t *pin) {
    pin->port->ODR ^= 1 << pin->bit;
}

static inline void gpio_pin_assign(Pin_t *pin, bool value) {
    pin->port->BSRR = 1 << (value ? pin->bit : pin->bit + 16);
}

static inline void gpio_pull_down(GPIO_TypeDef *port, uint32_t pin) {
  port->BSRR = 1 << (pin + 16); // Clear bit in ODR
}

static inline void gpio_pull_up(GPIO_TypeDef *port, uint32_t pin) {
  port->BSRR = 1 << pin; // Set bit in ODR
}

static inline void gpio_pin_pull_up(Pin_t *pin) {
  pin->port->BSRR = 1 << pin->bit; // Set bit in ODR
}

