#pragma once


void usart_rx_start(USART_TypeDef *dev, uint8_t *rx_buffer, uint32_t rx_buffer_size);

bool usart_rx_input_pending(const USART_TypeDef *dev);
bool usart_rx_eol(const USART_TypeDef *dev);

uint8_t *usart_rx_read_line(USART_TypeDef *dev, uint8_t *buffer, uint32_t buffer_len, float timeout);
