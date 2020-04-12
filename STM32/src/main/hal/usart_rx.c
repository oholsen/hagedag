#include "board/board.h"
#include "hal/systick.h"
#include "hal/usart.h"
#include "hal/usart_rx.h"
#include <string.h>
#include <assert.h>


typedef struct {
    uint8_t *buffer;
    uint32_t size;
    uint32_t tail;
    bool eol;
} usart_rx_t;

static volatile usart_rx_t buffers[3];



static inline void rx_append(volatile usart_rx_t *rx, uint8_t byte)
{
    // Not really required, as must be started for irq to call here
    if (rx->buffer == NULL) 
        return;

    if (byte == '\n')
      rx->eol = true;
    
    uint32_t tail = rx->tail;
    uint32_t size = rx->size;
    if (tail < size)
        rx->buffer[tail++] = byte;
    rx->tail = tail;
}     
                                 
static inline volatile usart_rx_t * rx_buffer(const USART_TypeDef *dev)
{
    if (dev == USART1)
        return &buffers[0];
    if (dev == USART2)
        return &buffers[1];
    if (dev == USART3)
        return &buffers[2];
    return NULL;
}

bool usart_rx_input_pending(const USART_TypeDef *dev) 
{
    volatile usart_rx_t *rx = rx_buffer(dev);
    return rx != NULL && rx->tail != 0;
}

bool usart_rx_eol(const USART_TypeDef *dev)
{
    volatile usart_rx_t *rx = rx_buffer(dev);
    return rx != NULL && rx->eol;
}

static void start(IRQn_Type irq_n, int irq_pri)
{
    NVIC_SetPriority(irq_n, 2);
    NVIC_EnableIRQ(irq_n);
}


void usart_rx_start(USART_TypeDef *dev, uint8_t *buffer, uint32_t buffer_size)
{
    volatile usart_rx_t *rx = rx_buffer(dev);
    assert(rx != NULL);
    rx->buffer = buffer;
    rx->size = buffer_size;
    rx->tail = 0;
    rx->eol = false;

    if (dev == USART1)
        start(USART1_IRQn, 3);
    else if (dev == USART2)
        start(USART2_IRQn, 2);
    else if (dev == USART3)
        start(USART3_IRQn, 2);
}


// TODO: do the interrupt vector trick pointing to same dispatch routine???
// Or - function call to different RX buffers...


#pragma call_graph_root = "interrupt" 
__irq  
void USART1_IRQHandler(void)
{
    if ((USART1->SR & USART_SR_RXNE) != 0)
    {
        uint8_t ch = (uint8_t) USART1->DR;
        rx_append(&buffers[0], ch);
    }
}


// Block until newline or timeout. returns trailing newline in buffer.
// TODO: have usart_rx_line_ready() triggered by newline (timeout on garbage: timestamp first char in line)
// TODO: set up line reader and poll for ready (or soft interrupt)
uint8_t *usart_rx_read_line(USART_TypeDef *dev, uint8_t *buffer, uint32_t buffer_len, float timeout)
{
    volatile usart_rx_t *rx = rx_buffer(dev);
    if (rx == NULL)
        return NULL;

    uint32_t i = 0; // in buffer
    uint32_t j = 0; // in uart_rx_buffer, # bytes consumed
    uint32_t expiration = systick_expiration(systick_seconds2ticks(timeout));
    
    for (;;) {

      if (systick_expired(expiration))
           break;
      
       if (j == rx->tail) 
           continue;

       uint8_t ch = rx->buffer[j++];
       
       // reserve space for NUL
       if (i + 1 < buffer_len)
            buffer[i++] = ch;
       
       if ((ch == '\n')) 
           break;
   }
   
   // FIXME: figure out IRQ from dev!?!?! Or embed IRQ in usart_rx_t.
   NVIC_DisableIRQ(USART1_IRQn); // FIXME
   void * dest = (void *) rx->buffer;
   const void * src = (const void *) (rx->buffer + j);
   memcpy(dest, src, rx->tail - j);
   rx->tail -= j;
   rx->eol = false; 
   // check if \n in rx buffer[0, rx->tail)
   for (int i = 0; i < rx->tail; i++) {
     if (rx->buffer[i] == '\n') {
       rx->eol = true;
       break;
     }
   }
   NVIC_EnableIRQ(USART1_IRQn); // FIXME

   // terminate output buffer with NUL
   buffer[i] = 0;
   return buffer;
}
