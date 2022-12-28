#include <avr/io.h>
#include <avr/interrupt.h>

#include "uart.h"

#define FOSC 16000000UL
#define UART_UBRR_115200 (FOSC/8/115200UL-1) /* U2X set to 1*/
#define UART_UBRR_500K (3)
#define UART_UBRR_1M (1)
#define UART_UBRR_2M (0)


void 
uart_init(uint8_t spd) 
{
  uart_set_speed(spd);
  
  UCSR0A = _BV(U2X0);
  UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); /* 8N1 */
  UCSR0B = _BV(RXEN0) | _BV(TXEN0) | _BV(RXCIE0) | _BV(UDRIE0);
}

void
uart_set_speed(uint8_t spd)
{
 switch(spd) {
         default:
                 UBRR0 = (uint16_t)UART_UBRR_115200;
                 break;
         case UART_500K:
                 UBRR0 = (uint16_t)UART_UBRR_500K;
                 break;
         case UART_1M:
                 UBRR0 = (uint16_t)UART_UBRR_1M;
                 break;
         case UART_2M:
                 UBRR0 = (uint16_t)UART_UBRR_2M;
                 break;
 }
}

static volatile uint8_t rx_rp = 0;
static volatile uint8_t rx_wp = 0;
static volatile uint8_t rx_ring[UART_RX_FIFO_SIZE];
static volatile uint8_t tx_rp = 0;
static volatile uint8_t tx_wp = 0;
static volatile uint8_t tx_ring[UART_TX_FIFO_SIZE];
static volatile uint8_t esc = 0;

ISR(USART_RX_vect)
{
 uint8_t prev, next, b;

 if(UCSR0A & _BV(FE0)) {
   (void) UDR0;
   return;
 }

 b = UDR0;
 if(b == 27) esc = 1;
 prev = rx_wp;
 next = prev+1;
 if(next == UART_RX_FIFO_SIZE) next = 0;
 if(next == rx_rp) return; /* overrun */
 rx_ring[prev] = b;
 rx_wp = next;
}

ISR(USART_UDRE_vect)
{
 uint8_t next;

 next = tx_rp;
 if(next == tx_wp) UCSR0B &= ~_BV(UDRIE0);
 else {
  UDR0 = tx_ring[next];
  if(++next == UART_TX_FIFO_SIZE) next = 0;
  tx_rp = next;
 }
}

uint8_t
uart_rx_esc_char(void)
{
 if(esc) {
   esc = 0;
   return 1;
 }
 return 0;
}

void 
uart_tx(uint8_t b) 
{
  uint8_t prev, next;
  prev = tx_wp;
  next = prev+1;
  if(next == UART_TX_FIFO_SIZE) next = 0;
  while(next == tx_rp);
  tx_ring[prev] = b;
  tx_wp = next;
  UCSR0B |= _BV(UDRIE0);
}

uint8_t
uart_rx_count(void)
{
 int8_t s;
 s = rx_wp-rx_rp;
 if(s < 0) s += UART_RX_FIFO_SIZE;
 return s;
}

uint8_t
uart_rx_empty(void)
{
 return rx_wp==rx_rp;
}

uint8_t
uart_tx_empty(void)
{
 /* TODO: use TXC interrupt to know that nothing's being transmitted? */
 return tx_wp==tx_rp;
}

uint8_t 
uart_rx(void)
{
  uint8_t b;
  uint8_t next = rx_rp;
  while (rx_wp == next);
  b = rx_ring[next];
  if(++next == UART_RX_FIFO_SIZE) next = 0;
  rx_rp = next;
  return b;
}

uint8_t 
uart_peek(void)
{
  return rx_ring[rx_rp];
}
