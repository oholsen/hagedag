/**
 * Redirect IAR´s printf output to our own UART
 *
 * This means implementing an __write() function call which will
 * be given "priority" by the linker over IARs built in routines.
 *
 */

#include <stddef.h>
#include "board/board.h"
#include "hal/uart_tx.h"
#include "hal/usart.h"

static inline USART_TypeDef *handle2usart(int handle)
{
    switch (handle) {
#if defined(DEBUG_UART)
    case 1:
    case 2:
        return DEBUG_UART;
#endif
    case 11: return USART1;
    case 12: return USART2;
    case 13: return USART3;
    }
    return NULL;
}

// This implementation is based on the example given in the
// IAR C/C++ Development Guide (EWARM_DevelopmentGuide.ENU.pdf)

size_t __write(int handle, const unsigned char *buf, size_t bufSize)
{
    /* Check for the command to flush all handles */
    if (handle == -1) {
        return 0;
    }
    /* Check for stdout and stderr (only necessary if FILE descriptors are enabled.) */
    if (handle != 1 && handle != 2) {
        return -1;
    }
#if 0
    USART_TypeDef *usart = handle2usart(handle);
    if (usart == NULL)
        return -1;
    usart_tx_raw(usart, buf, bufSize);
#endif
#ifdef STDIO_UART
    usart_tx_raw(STDIO_UART, buf, bufSize);
#endif
    
    // TODO: this is required for robobob: copy stdout to Rasperry Pi for web app control and diagnostics
    // Preferably sprintf and send to two USARTs - Or configure "tee" here.
    //usart_tx_raw(USART3, buf, bufSize);
    
    return bufSize;
}

int __open(const char * filename, int mode)
{
    return -1;
}

size_t __read(int handle, unsigned char *buf, size_t bufSize)
{
    size_t nChars = 0;
    
    /* Check for stdin (only necessary if FILE descriptors are enabled) */
    if (handle != 0) {
        return -1;
    }
    
    for (/*Empty*/; bufSize > 0; --bufSize)
    {
        unsigned char c = UartRead();
        if (c == 0)
            break;
        *buf++ = c;
        ++nChars;
    }
    return nChars;
}

int __close(int handle)
{
  return 0;
}

long __lseek(int handle, long offset, int whence)
{
  return -1;
}

int remove(const char * filename)
{
  return 0;
}