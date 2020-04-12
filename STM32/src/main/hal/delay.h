#pragma once

extern void delay_us(uint32_t);

static inline void delay_ms(uint32_t ms)
{
    delay_us(1000 * ms);
}
