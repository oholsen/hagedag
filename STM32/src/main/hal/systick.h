#pragma once


#define SYSTICK_CLOCK 8000000

/*
24-bit systick counter runs at system clock. Wrap-around at 16M, i.e. every 4 seconds.
Counts DOWN!!!
*/
static inline void systick_start(void)
{
  SysTick->LOAD  = SysTick_LOAD_RELOAD_Msk;                                  /* set reload register */
  //NVIC_SetPriority (SysTick_IRQn, (1<<__NVIC_PRIO_BITS) - 1);  /* set Priority for Systick Interrupt */
  SysTick->VAL   = 0;                                          /* Load the SysTick Counter Value */
  // Use the reference clock HCLK / 8 (can also use HCLK/processor clock
  SysTick->CTRL  = //SysTick_CTRL_CLKSOURCE_Msk | // processor clock
                   //SysTick_CTRL_TICKINT_Msk   |
                   SysTick_CTRL_ENABLE_Msk;
  SysTick->LOAD  = SysTick_LOAD_RELOAD_Msk;
}

static inline void systick_stop(void)
{
  SysTick->CTRL = 0;
}

static inline uint32_t systick_read(void)
{
    return SysTick->VAL;
}

static inline uint32_t systick_elapsed(uint32_t from, uint32_t to)
{
    return (from - to) & SysTick_VAL_CURRENT_Msk;
}

static inline float systick_ticks2seconds(uint32_t ticks)
{
    return ((float) ticks) / SYSTICK_CLOCK;
}

static inline uint32_t systick_seconds2ticks(float dt)
{
    return (uint32_t) (dt * SYSTICK_CLOCK);
}

static inline float systick_elapsed_seconds(uint32_t from, uint32_t to)
{
    return systick_ticks2seconds(systick_elapsed(from, to));
}

static inline uint32_t systick_add(uint32_t from, uint32_t delta)
{
    return (from - delta) & SysTick_VAL_CURRENT_Msk;
}

static inline uint32_t systick_expiration(uint32_t delta)
{
    return systick_add(systick_read(), delta);
}

/**
expiration is from systick_add() or systick_expiration().
maximum wait delay is half systick period.
*/
static inline bool systick_expired2(uint32_t t, uint32_t expiration)
{
    return systick_elapsed(t, expiration) >= (1 << 23);
}

/**
expiration is from systick_add() or systick_expiration().
maximum wait delay is half systick period.
*/
static inline bool systick_expired(uint32_t expiration)
{
    uint32_t now = systick_read();
    return systick_expired2(now, expiration);
}

