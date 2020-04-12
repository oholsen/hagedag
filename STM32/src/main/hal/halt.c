#include "board/board.h"
#include "board/led.h"
#include "hal/delay.h"


static inline void halt(int irq)
{
    // LED blinking identifies fault
    for(;;)
    {
        delay_ms(3000);
        for (int i = 0; i < irq; i++) 
        {
            led_on();
            delay_ms(500);
            led_off();
            delay_ms(500);
        }
    }
}


#pragma call_graph_root = "interrupt"
__irq void NMI_Handler(void)
{
    halt(1);
}

#pragma call_graph_root = "interrupt"
__irq void MemManage_Handler(void)
{
    halt(2);
}

#pragma call_graph_root = "interrupt"
__irq void HardFault_Handler(void)
{
    halt(3);
}

#pragma call_graph_root = "interrupt"
__irq void BusFault_Handler(void)
{
    halt(4);
}

#pragma call_graph_root = "interrupt"
__irq void UsageFault_Handler(void)
{
    halt(5);
}

#pragma call_graph_root = "interrupt"
__irq void SVC_Handler(void)
{
    halt(6);
}
