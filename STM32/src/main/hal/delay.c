#include "board/board.h"
#include "hal/systick.h"
#include "hal/delay.h"

/**
Systick timer wraps around in a couple of seconds.
*/
// delay 1s without wrap-around
#define DELAY_TICKS ((1 << 20) - 1)

static uint32_t _delay_ticks(uint32_t t, uint32_t ticks)
{
    uint32_t expiration = systick_add(t, ticks);
    while (!systick_expired(expiration));
    return expiration;
}

static void delay_ticks(uint32_t ticks)
{
    uint32_t t = systick_read();
    while (ticks > DELAY_TICKS) {
        t = _delay_ticks(t, DELAY_TICKS);
        ticks -= DELAY_TICKS;
    }
    _delay_ticks(t, ticks);
}

void delay_us(uint32_t us)
{

#if SYSTICK_CLOCK == 8000000
     uint32_t ticks = 64 * us;
#else
#error SYSTICK_CLOCK not supported
#endif

    delay_ticks(ticks);
}
