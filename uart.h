#pragma once
#define UART_TX_FIFO_SIZE 64
#define UART_RX_FIFO_SIZE 64

void uart_init(uint8_t spd);
void uart_tx(uint8_t b);
uint8_t uart_tx_empty(void);
uint8_t uart_rx_count(void);
uint8_t uart_rx_empty(void);
uint8_t uart_rx_esc_char(void);
uint8_t uart_rx(void);
uint8_t uart_peek(void);

#define UART_115200 0
#define UART_500K   2
#define UART_1M     3
#define UART_2M     4
void uart_set_speed(uint8_t spd);
