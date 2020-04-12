#pragma once


void usart_start(USART_TypeDef *dev, float baud_rate);
void usart_stop(USART_TypeDef *dev);


void usart_tx(USART_TypeDef *dev, uint8_t b);

uint8_t usart_rx(USART_TypeDef *dev);

void usart_tx_complete(USART_TypeDef *dev);


static inline bool usart_tx_ready(USART_TypeDef *dev)
{
  return (dev->SR & USART_SR_TXE) != 0;
}

static inline bool usart_rx_ready(USART_TypeDef *dev)
{
  return (dev->SR & USART_SR_RXNE) != 0;
}


void usart_tx_raw(USART_TypeDef *dev, const uint8_t *data, int len);

void usart_tx_string(USART_TypeDef *dev, const char *str);
