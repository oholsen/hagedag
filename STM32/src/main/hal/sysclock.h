#pragma once

#warning here

#define LSE_CLOCK                  32768
#define LSI_CLOCK                  37000
#define MSI_CLOCK                4194000 // MSI as per RCC_ICSCR spec
#define HSI16_CLOCK             16000000
#define HSI16DIV4_CLOCK         (HSI16_CLOCK / 4)
#define STLINK_CLOCK             8000000 // External clock from ST-Link


#define SYSTEM_TICKS_US(us)    ((uint32_t) (1e-6 * SYSTEM_CLOCK * (us)))
#define SYSTEM_TICKS_MS(ms)    ((uint32_t) (1e-3 * SYSTEM_CLOCK * (ms)))

void sysclock_default(void);

void sysclock_external(void);


// Returns system clock ticks in the interval defined by the lptimer
// running on the LSE oscillator.
extern uint32_t sysclock_measure(uint32_t lptimer_ticks);

// Only works for HSI16/4
extern void sysclock_calibrate(void);

