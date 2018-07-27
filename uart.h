#pragma once
#define UART_BAUD 115200UL
#define UART_TX_FIFO_SIZE 64
#define UART_RX_FIFO_SIZE 64

void uart_init(void);
void uart_tx(uint8_t b);
uint8_t uart_rx_count(void);
uint8_t uart_rx_empty(void);
uint8_t uart_rx_esc_char(void);
uint8_t uart_rx(void);
