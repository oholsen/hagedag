#include "board/board.h"
#include "hal/usart.h"


void usart_start(USART_TypeDef *dev, float baud_rate)
{    
    if (dev == USART1)
    {
        RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    }
    else if (dev == USART2)
    {
        RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    }
    else if (dev == USART3)
    {
        RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
    }
    else
        ; // TODO: assert

    dev->BRR = (uint32_t) (SYSTEM_CLOCK / baud_rate);
    dev->CR1 = USART_CR1_TE | USART_CR1_UE | USART_CR1_RE | USART_CR1_RXNEIE;
}


void usart_stop(USART_TypeDef *dev)
{   
    dev->CR1 = 0;

    if (dev == USART1)
        RCC->APB2ENR &= ~RCC_APB2ENR_USART1EN;
    else if (dev == USART2)
        RCC->APB1ENR &= ~RCC_APB1ENR_USART2EN;
    else if (dev == USART3)
        RCC->APB1ENR &= ~RCC_APB1ENR_USART3EN;
    else
        ; // TODO: assert
}


void usart_tx(USART_TypeDef *dev, uint8_t b)
{
    while (!usart_tx_ready(dev));
    dev->DR = (uint16_t) b;
}


uint8_t usart_rx(USART_TypeDef *dev)
{
    while (!usart_rx_ready(dev));
    return (uint8_t) dev->DR;
}


void usart_tx_complete(USART_TypeDef *dev)
{
    while ((dev->SR & USART_SR_TC) == 0);
}


void usart_tx_raw(USART_TypeDef *dev, const uint8_t *data, int len)
{
    while (len-- > 0)
    {
        usart_tx(dev, *data++);
    }
}


void usart_tx_string(USART_TypeDef *dev, const char *str)
{
    while (*str != 0)
    {
        usart_tx(dev, *str);
        str++;
    }
}

