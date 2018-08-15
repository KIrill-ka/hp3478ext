/*
  HP3478EXT is intended to extend functionality of HP3478A digital multimeter.
  When connected to GPIB bus, it waits for SRQ messages to activate extra modes.
  
  Also it works as a general purpose GPIB <-> UART converter. The command set and
  line editing are derived from Jacek Greniger's gpib converter project 
  (github.com/JacekGreniger/gpib-converter).

  The hardware is ATmega 328 "arduino nano style" board.
 */

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <ctype.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#define F_CPU 16000000UL  
#include <util/delay.h>

#include "uart.h"


/*
 TODO list

 - Display 0 with O?
 - fix ext mode (gpib_transmit & gpib_receive interface change)
 - Implement O0 = reset to defaults
 - Implement unbuffered binary write TUD using escape-sequence as stop
 - Implement unbuffered binary read TUD using escape-sequence as stop
 */

/* 
 Known caveats

 - hp3478 commands can be interrupted with ATN. For some reason, if the byte
   have been accepted by GPIB interface, it doesn't mean that the command will
   be processed. As a workaround, dummy <LF> bytes are sent so gpib_transmit waits 
   for RFD after the command.
 - There are different ways to clear SRQ signal and status bits, and they are all
   have some "features".
   1) K and M commands deactivate SRQ ~250uS after the command completes.
   2) Read deactivates SRQ ~250uS after it's started.
   3) Serial poll clears SRQ immediately, but status bits are left for some time.
   4) K does not clear status bits immediately, but faster and more reliable than serial poll.
*/

#define CMD_HISTORY_SIZE 8
#define CMD_BUF_SIZE 64

#define GPIB_TALK_ADDR_OFFSET 64
#define GPIB_LISTEN_ADDR_OFFSET 32

/* GPIB status byte / SRQ mask / status byte 3 */
#define HP3478_SB_DREADY (1<<0)
#define HP3478_SB_SYNERR (1<<2)
#define HP3478_SB_INTERR (1<<3)
#define HP3478_SB_FRPSRQ (1<<4)
#define HP3478_SB_INVCAL (1<<5)
#define HP3478_SB_SRQMSG (1<<6)
#define HP3478_SB_PWRSRQ (1<<7)

/* status byte 1 */
#define HP3478_ST_N_DIGITS  (3<<0)
#define HP3478_ST_N_DIGITS5 (1<<0)
#define HP3478_ST_N_DIGITS4 (2<<0)
#define HP3478_ST_N_DIGITS3 (3<<0)

#define HP3478_ST_RANGE     (7<<2)
#define HP3478_ST_RANGE1    (1<<2) /* 30mV DC, 300mV AC, 30 ohm, 300mA AC or DC, Extended Ohms */
#define HP3478_ST_RANGE2    (2<<2) /* 300mV DC, 3V AC, 300 ohm 3A AC or DC */
#define HP3478_ST_RANGE3    (3<<2) /* 3V DC, 30V AC, 3K ohm */
#define HP3478_ST_RANGE4    (4<<2) /* 30V DC, 300V AC, 30K ohm */
#define HP3478_ST_RANGE5    (5<<2) /* 300V DC, 300K ohm */
#define HP3478_ST_RANGE6    (6<<2) /* 3M ohm */
#define HP3478_ST_RANGE7    (7<<2) /* 30M ohm */

#define HP3478_ST_FUNC      (7<<5)
#define HP3478_ST_FUNC_DCV  (1<<5)
#define HP3478_ST_FUNC_ACV  (2<<5)
#define HP3478_ST_FUNC_2WOHM (3<<5)
#define HP3478_ST_FUNC_4WOHM (4<<5)
#define HP3478_ST_FUNC_DCA  (5<<5)
#define HP3478_ST_FUNC_ACA  (6<<5)
#define HP3478_ST_FUNC_XOHM (7<<5)

/* status byte 2 */
#define HP3478_ST_INT_TRIGGER (1<<0)
#define HP3478_ST_AUTORANGE   (1<<1)
#define HP3478_ST_AUTOZERO    (1<<2)

#define SET_PORT_PIN(PORT, PIN, V) if(V) PORT |= PIN; else PORT &= ~PIN
#define CAT(a, b) a ## b
#define PORT(X) CAT(PORT, X)
#define DDR(X) CAT(DDR, X)
#define PIN(X) CAT(PIN, X)

/* configuration constants & defaults */
#define GPIB_MY_DEFAULT_ADDRESS 21
#define GPIB_HP3478_DEFAULT_ADDRESS 23
#define GPIB_BUF_SIZE 127
#define GPIB_MAX_RECEIVE_TIMEOUT_mS 200
#define GPIB_MAX_TRANSMIT_TIMEOUT_mS 200

/* PIN assignment */
/*
 GPIB|       |                    |            | atmega  |
 pin | name  | description        | direction  | pin     |    
 ----+-------+--------------------+------------+---------+
 1   | DIO1  | Data bit 1 (LSB)   | Talker     | PD2  32 | 2
 2   | DIO2  | Data bit 2         | Talker     | PD3   1 | 3
 3   | DIO3  | Data bit 3         | Talker     | PD4   2 | 4
 4   | DIO4  | Data bit 4         | Talker     | PD5   9 | 5
 5   | EOI   | End Of Indentity   | Talker     | PB3  15 | 11
 6   | DAV   | Data Valid         | Controller | PB4  16 | 12
 7   | NRFD  | Not Ready For Data | Listener   | PC0  23 | A0
 8   | NDAC  | No Data Accepted   | Listener   | PC1  24 | A1
 9   | IFC   | Interface Clear    | Controller | PC2  25 | A2
 10  | SRQ   | Service Request    | Slave      | PC3  26 | A3
 11  | ATN   | Attention          | Controller | PC4  27 | A4
 12  |       | Shield             |            |         |
 13  | DIO5  | Data bit 5         | Talker     | PD6  10 | 6
 14  | DIO6  | Data bit 6         | Talker     | PD7  11 | 7
 15  | DIO7  | Data Bit 7         | Talker     | PB0  12 | 8
 16  | DIO8  | Data bit 8 (MSB)   | Talker     | PB1  13 | 9
 17  | REN   | Remote Enabled     | Controller | PC5  28 | A5
 18  |       | GND DAV            |            |         |
 19  |       | GND NRFD           |            |         |
 20  |       | GND NDAC           |            |         |
 21  |       | GND IFC            |            |         |
 22  |       | GND SRQ            |            |         |
 23  |       | GND ATN            |            |         |
 24  |       | GND data           |            |         |

     | LED   |                    | OUT        | PB5     |
     |BUZZER | Buzzer PWM (OC1B)  | OUT        | PB2     |
*/

#define EOI  _BV(PB3)
#define EOI_PORT B
#define DAV  _BV(PB4)
#define DAV_PORT B
#define NRFD _BV(PC0)
#define NRFD_PORT C
#define NDAC _BV(PC1)
#define NDAC_PORT C
#define IFC  _BV(PC2)
#define IFC_PORT C
#define SRQ  _BV(PC3)
#define SRQ_PORT C
#define ATN  _BV(PC4)
#define ATN_PORT C
#define REN  _BV(PC5)
#define REN_PORT C

#define LED _BV(PB5)
#define LED_PORT B

static void cfg_data_in(void) {
  DDRD &= ~(_BV(PD2)|_BV(PD3)|_BV(PD4)|_BV(PD5)|_BV(PD6)|_BV(PD7));
  DDRB &= ~(_BV(PB0)|_BV(PB1));
}
static void cfg_data_out(void) {
  DDRD |= (_BV(PD2)|_BV(PD3)|_BV(PD4)|_BV(PD5)|_BV(PD6)|_BV(PD7));
  DDRB |= (_BV(PB0)|_BV(PB1));
}

static uint8_t data_get(void) {
  uint8_t d;
  d = PIND;
  d >>= 2;
  if(PINB & _BV(PB0)) d |= 64;
  if(PINB & _BV(PB1)) d |= 128;
  return ~d;
}

static void data_put(uint8_t d) {
  DDRD = (DDRD&0x3) | (d<<2);
  SET_PORT_PIN(DDRB, _BV(PB0), d&64);
  SET_PORT_PIN(DDRB, _BV(PB1), d&128);
}

static inline void eoi_set(x) {SET_PORT_PIN(DDR(EOI_PORT), EOI, (x));}
static inline void dav_set(x) {SET_PORT_PIN(DDR(DAV_PORT), DAV, (x));}
static inline void nrfd_set(x) {SET_PORT_PIN(DDR(NRFD_PORT), NRFD, (x));}
static inline void ndac_set(x) {SET_PORT_PIN(DDR(NDAC_PORT), NDAC, (x));}
static inline void SetIFC(x) {SET_PORT_PIN(DDR(IFC_PORT), IFC, !(x));}
static inline void set_atn(x) {
 SET_PORT_PIN(DDR(ATN_PORT), ATN, (x));
 if(x) _delay_us(0.5); /* T7 in ieee488 spec */
}
static inline void set_ren(x) {SET_PORT_PIN(DDR(REN_PORT), REN, (x));}

static inline uint8_t dav(void) {return !(PIN(DAV_PORT) & DAV);}
static inline uint8_t ndac(void) {return !(PIN(NDAC_PORT) & NDAC);}
static inline uint8_t nrfd(void) {return !(PIN(NRFD_PORT) & NRFD);}
static inline uint8_t srq(void) {return !(PIN(SRQ_PORT) & SRQ);}
static inline uint8_t eoi(void) {return !(PIN(EOI_PORT) & EOI);}
static inline uint8_t ren(void) {return (DDR(REN_PORT) & REN);}

const char help[] PROGMEM = 
  "\r\n"
  "hp3478ext GPIB-UART converter\r\n"
  "Transmission commands\r\n"
  "  C  Send ASCII command\r\n"
  "  D  Send/receive ASCII data\r\n"
  "  THC Send HEX command\r\n"
  "  THD Send*/receive** HEX data\r\n"
  "  TBD Send/receive* HEX data\r\n"
  "  P Continous read (plotter mode), <ESC> to exit\r\n"
  "GPIB control\r\n"
  "  R Set REMOTE mode (REN true)\r\n"
  "  L Set LOCAL mode (REN false)\r\n"
  "  I Generate IFC pulse\r\n"
  "Other commands\r\n"
  "  S Get REN/SRQ/LISTEN state (1 if true)\r\n"
  "  O Get/set an option (O? for list)\r\n"
  "  H Command history\r\n\r\n"
  "* Add ; at the end to disable EOI\r\n"
  "** You can specify length in hex after the command (up to 7f)\r\n\r\n"
;
const char opt_help[] PROGMEM = 
  "\r\n"
  "O<opt> Show current value\r\n"
  "O<opt><val> Set option value\r\n"
  "O<opt><val>w Set option value and write to EEPROM\r\n"
  "<opt>:\r\n"
  "  I Interactive mode (0 off, 1 on)\r\n"
  "  C Converter GPIB address\r\n"
  "  D HP3478A GPIB address\r\n"
  "  T Transmit end of line*\r\n"
  "  R Receive end of line*\r\n"
  "  X HP3478A extension mode (0 off, 1 on)\r\n"
  "  B Baud rate (0=115200, 2=500K)\r\n"
  "  0 Set defaults for interactive operation\r\n"
  "  1 Set defaults for non interactive\r\n\r\n"
  "* ORed bits: 4=EOI, 2=<LF>, 1=<CR>\r\n\r\n"
;


#define GPIB_END_CR  1
#define GPIB_END_LF  2
#define GPIB_END_EOI 4
#define GPIB_END_BUF 8 /* synthetic, used as return value from gpib_receive */

uint8_t EEMEM hp3478_ext_en_eep = 1;
uint8_t EEMEM uart_echo_eep = 1;
uint8_t EEMEM gpib_my_addr_eep = GPIB_MY_DEFAULT_ADDRESS;
uint8_t EEMEM gpib_hp3478_addr_eep = GPIB_HP3478_DEFAULT_ADDRESS;
uint8_t EEMEM dummy1_eep = 0;
uint8_t EEMEM gpib_end_seq_rx_eep = GPIB_END_EOI;
uint8_t EEMEM gpib_end_seq_tx_eep = GPIB_END_EOI;
uint8_t EEMEM uart_baud_eep = UART_115200;

#define GPIB_LISTEN 1
#define GPIB_TALK   2
static uint8_t gpib_state = 0;
static uint8_t gpib_end_seq_tx;
static uint8_t gpib_end_seq_rx;
static uint8_t gpib_my_addr;
static uint8_t gpib_hp3478_addr;
volatile uint8_t gpib_srq_interrupt;


static char cmd_hist[CMD_BUF_SIZE*CMD_HISTORY_SIZE];
static char cmd_hist_len = 0;

enum led_mode {LED_OFF, LED_SLOW, LED_FAST};
static enum led_mode led_state;

static uint8_t hp3478_ext_enable;

static uint8_t uart_echo;
static uint8_t uart_baud = UART_115200;

volatile uint16_t msec_count;

#define EV_TIMEOUT     1
#define EV_SRQ         2
#define EV_UART        4
#define EV_EXT_DISABLE 8
#define EV_EXT_ENABLE 16

#define TIMEOUT_INF   0xffff
#define TIMEOUT_CONT  0xfffe

static void set_defaults(uint8_t set);

static void 
led_set(enum led_mode m) 
{
  if(m == LED_OFF) PORT(LED_PORT) &= ~LED;
  led_state = m;
}
static inline void led_toggle(void) {PIN(LED_PORT) |= LED;}

static uint8_t buzzer = 0;
static void 
beep(uint8_t on)
{
 buzzer = on;
 if(on) {
  DDRB |= _BV(PB2);
  TCCR1B |= _BV(CS10);
 } else {
  TCCR1B &= ~_BV(CS10);
  DDRB &= ~_BV(PB2); /* Occasionally some phantom signal remaining on the 
                        pin after timer stopped. Disconnecting it does help. */
 }
}
static int 
uart_putchar(char ch, FILE* file)
{
  uart_tx(ch);
  return 0;
}

static void 
gpib_listen(void)
{
  cfg_data_in();

  PORT(DAV_PORT) |= DAV; /* enable pullup on DAV so reads won't return garbage */

  /* OUTPUTS: IFC | ATN | REN | NRFD | NDAC */
  nrfd_set(1);
  ndac_set(1);
}

static void 
gpib_talk(void)
{
  cfg_data_out();
  
  PORT(DAV_PORT) &= ~DAV;

  /* OUTPUTS: IFC | ATN | REN | EOI | DAV */
  nrfd_set(0);
  ndac_set(0);
}


static uint8_t
gpib_receive(uint8_t *buf, uint8_t buf_size, uint8_t *n_received, uint8_t stop)
{
  uint8_t index = 0;
  uint8_t c;
  uint8_t do_stop = 0;
  uint8_t ts;

  do {
    nrfd_set(0); /* ready for receiving data */
    
    ts = (uint8_t)msec_count;
    while (!dav()) { /* waiting for falling edge */
      if ((uint8_t)((uint8_t)msec_count-ts) > GPIB_MAX_RECEIVE_TIMEOUT_mS) {
        *n_received = index;
        nrfd_set(1);
        return 0;
      }
    }
    
    nrfd_set(1); /* not ready for receiving data */
    if (eoi() && (stop & GPIB_END_EOI) != 0) do_stop = GPIB_END_EOI;
    
    c = data_get();
    ndac_set(0); /* data accepted */

    buf[index++] = c;
    if (c == 10 && (stop & GPIB_END_LF) != 0) do_stop |= GPIB_END_LF;
    if (c == 13 && (stop & GPIB_END_CR) != 0) do_stop |= GPIB_END_CR;

    while (dav()) { /* waiting for rising edge */
      if ((uint8_t)((uint8_t)msec_count-ts) > GPIB_MAX_RECEIVE_TIMEOUT_mS) {
        *n_received = index;
        ndac_set(1);
        return 0;
      }
    }
    
    ndac_set(1);
  } while (index < buf_size && !do_stop);
  *n_received = index;
  if(do_stop) return do_stop;
  return GPIB_END_BUF;
}

static uint8_t
gpib_transmit(const uint8_t *buf, uint8_t len, uint8_t end)
{
  uint8_t i;
  uint8_t ts;
  
  if (!nrfd() && !ndac()) return 0;

  if(end & GPIB_END_LF) len++;
  if(end & GPIB_END_CR) len++;
  
  for(i = 0; i < len; i++) {
    
    uint8_t d;
    if((end & (GPIB_END_LF|GPIB_END_CR)) == (GPIB_END_CR|GPIB_END_LF) && i == len-2) d = 13;
    else if((end & (GPIB_END_LF|GPIB_END_CR)) == GPIB_END_CR && i == len-1) d = 13;
    else if((end & GPIB_END_LF) != 0 && i == len-1) d = 10;
    else d = buf[i];

    data_put(d);
    if (i == len-1 && (end & GPIB_END_EOI) != 0) eoi_set(1);
    
    _delay_us(2); /* T1 in ieee488 spec */
     
    ts = (uint8_t)msec_count;
    while (nrfd()) { /* waiting for high on NRFD */
      if ((uint8_t)((uint8_t)msec_count-ts) > GPIB_MAX_TRANSMIT_TIMEOUT_mS) {
        eoi_set(0);
        return i;
      }
    }
    
    dav_set(1);
   
    while (ndac()) { /* waiting for high on NDAC */
      if ((uint8_t)((uint8_t)msec_count-ts) > GPIB_MAX_TRANSMIT_TIMEOUT_mS) {
        eoi_set(0);
        dav_set(0);
        return i;
      }
    }
    
    dav_set(0);
  }
  eoi_set(0);

  return i;
}

static uint8_t
gpib_transmit_b(const uint8_t *buf, uint8_t len, uint8_t end)
{
  uint8_t l = len;
  if(end & GPIB_END_LF) l++;
  if(end & GPIB_END_CR) l++;
  return gpib_transmit(buf, len, end) == l;
}

static uint8_t
gpib_transmit_P(const uint8_t *buf, uint8_t len, uint8_t end)
{
  uint8_t i;
  uint8_t ts;
  
  if (!nrfd() && !ndac()) return 0;

  if(end & GPIB_END_LF) len++;
  if(end & GPIB_END_CR) len++;
  
  for(i = 0; i < len; i++) {
    
    uint8_t d;
    if((end & (GPIB_END_LF|GPIB_END_CR)) == (GPIB_END_CR|GPIB_END_LF) && i == len-2) d = 13;
    else if((end & (GPIB_END_LF|GPIB_END_CR)) == GPIB_END_CR && i == len-1) d = 13;
    else if((end & GPIB_END_LF) != 0 && i == len-1) d = 10;
    else d = pgm_read_byte(buf+i);

    data_put(d);
    if (i == len-1 && (end & GPIB_END_EOI) != 0) eoi_set(1);
    
    _delay_us(2); /* T1 in ieee488 spec */
     
    ts = (uint8_t)msec_count;
    while (nrfd()) { /* waiting for high on NRFD */
      if ((uint8_t)((uint8_t)msec_count-ts) > GPIB_MAX_TRANSMIT_TIMEOUT_mS) {
        eoi_set(0);
        return 0;
      }
    }
    
    dav_set(1);
   
    while (ndac()) { /* waiting for high on NDAC */
      if ((uint8_t)((uint8_t)msec_count-ts) > GPIB_MAX_TRANSMIT_TIMEOUT_mS) {
        eoi_set(0);
        dav_set(0);
        return 0;
      }
    }
    
    dav_set(0);
  }
  eoi_set(0);

  return 1;
}

ISR(TIMER0_OVF_vect) {
  static uint16_t led_timer;
  uint8_t l = led_state;
  msec_count++;
  if (l == LED_OFF) return;
	
  if (++led_timer >= (l == LED_SLOW ? 500 : 100)) {
    led_timer = 0;
    led_toggle();
  }
}

uint16_t srq_prev;
ISR(PCINT1_vect) {
 uint8_t s = srq();
 if(s != srq_prev) {
  // FIXME: added some filtering. Spurious SRQS are probably caused by capacitive coupling in long ribbon cable.
  gpib_srq_interrupt = 1;
  srq_prev = s;
 }
}

static inline uint16_t
msec_get(void)
{
 uint16_t v;
 cli();
 v = msec_count;
 sei();
 return v;
}

static uint8_t 
ishexdigit(uint8_t x)
{
 return (x >= '0' && x <= '9') || (x >= 'A' && x <= 'F') || (x >= 'a' && x <= 'f');
}

static uint8_t 
hex2dec(uint8_t x)
{
  if(x <= '9') return x-'0';
  if(x <= 'F') return x-'A'+10;
  return x-'a'+10;
}

static uint8_t 
convert_hex_message(const uint8_t *buf, uint8_t len, uint8_t *out, uint8_t *out_len, uint8_t *send_eoi)
{
  uint8_t i; 
  *send_eoi = GPIB_END_EOI;

  if (len < 2) return 0;
    
  if (';' == buf[len-1]) {
    len--;
    *send_eoi = 0;
  }

  if ((len & 0x01) != 0) return 0;
    
  for (i=0; i<len; i++) if (!ishexdigit(buf[i])) return 0;

  for (i=0; i<len; i+=2) {
    *out = (hex2dec(buf[i]) << 4) | hex2dec(buf[i+1]);
    out++;
  }
  *out_len = len>>1;

  return 1;
}

static void 
uart_puts(const uint8_t *s, uint8_t len)
{
 while(len) {
  uart_tx(*s);
  len--;
  s++;
 }
}

#define ESC_KEY_UP 0x41
#define ESC_KEY_DOWN 0x42
#define ESC_KEY_RIGHT 0x43
#define ESC_KEY_LEFT 0x44

static uint8_t 
line_edit(uint8_t c, uint8_t *buf, uint8_t *len)
{
 static uint8_t cursor;
 static uint8_t cmdlen;
 static uint8_t hist_pos;
#define LNEDIT_START 0
#define LNEDIT_NORM  1
#define LNEDIT_ESC   2
#define LNEDIT_ESC1  3
#define LNEDIT_DONE  4
 static uint8_t state = LNEDIT_START;

 uint8_t new_cmdlen;
 uint8_t i;
 uint8_t cmd;

 switch(state) {
         case LNEDIT_START:
                 if(uart_echo) printf_P(PSTR("<GPIB> "));
                 cursor = 0;
                 cmdlen = 0;
                 hist_pos = cmd_hist_len;
                 state = LNEDIT_NORM;
                 break;
         case LNEDIT_ESC:
                 if(c == 0x5b) {
                  state = LNEDIT_ESC1;
                 } else {
                  state = LNEDIT_NORM;
                 }
                 return 0;
         case LNEDIT_ESC1:
                 switch(c) {
                         case ESC_KEY_UP:
                         case ESC_KEY_DOWN:
                                 if(c == ESC_KEY_UP) {
                                  if(!hist_pos) break;
                                  hist_pos--;
                                  memmove(buf, cmd_hist+hist_pos*CMD_BUF_SIZE, CMD_BUF_SIZE);
                                  new_cmdlen = strlen((char*)buf);
                                 } else if(cmd_hist_len != 0 && hist_pos == cmd_hist_len-1) {
                                  hist_pos++;
                                  new_cmdlen = 0; 
                                 } else if(hist_pos+1 >= cmd_hist_len) {
                                  new_cmdlen = 0; /* allow to clear line by down-arrow */
                                 } else {
                                  hist_pos++;
                                  memmove(buf, cmd_hist+hist_pos*CMD_BUF_SIZE, CMD_BUF_SIZE);
                                  new_cmdlen = strlen((char*)buf);
                                 }
                                 while (cursor < cmdlen) {
                                  uart_tx(' ');
                                  cursor++;
                                 }
                                 while (cmdlen--) {
                                  uart_tx(0x08);
                                  uart_tx(' ');
                                  uart_tx(0x08);
                                 }
                                 uart_puts(buf, new_cmdlen);
                                 cmdlen = new_cmdlen;
                                 cursor = new_cmdlen;
                                 break;

                         case ESC_KEY_LEFT:
                                 if (cursor) {
                                  --cursor;
                                  uart_tx(0x1B);
                                  uart_tx(0x5B);
                                  uart_tx('D');
                                 }
                                 break;

                         case ESC_KEY_RIGHT:
                                 if (cursor < cmdlen) {
                                  cursor++;
                                  uart_tx(0x1B);
                                  uart_tx(0x5B);
                                  uart_tx('C');
                                 }
                                 break;
                 }
                 state = LNEDIT_NORM;
                 return 0;
 }

 cmd = 0;
 switch(c) {
          case 0x7f: /* del */
          case 0x08: /* backspace */
                  if(!uart_echo) break;
                  if(!cursor) break;
                  --cmdlen;
                  --cursor;
                  memmove(&buf[cursor], &buf[cursor+1], cmdlen-cursor);
                  uart_tx(0x08);
                  for (i=cursor; i < cmdlen; i++) uart_tx(buf[i]);
                  uart_tx(' ');
                  for (i=cursor; i < cmdlen+1; i++) uart_tx(0x08);
                  break;
          case 27: /* ESC */
                   if(!uart_echo) break;
                   state = LNEDIT_ESC;
                   break;
          case 10: break; /* LF */
          case 13:
                   if (uart_echo) {
                    uart_tx(13); /* CR */
                    uart_tx(10); /* LF */
                   }
                   if (cmdlen) {
                    cmd = toupper(buf[0]);
                    *len = cmdlen;
                    if(uart_echo && cmd != 'H') state = LNEDIT_DONE;
                    else state = LNEDIT_START;
                    break;
                   }
                   state = LNEDIT_START;
                   cmd = 13;
                   break;
          case 0:  break;
          default:
                   if (cmdlen == CMD_BUF_SIZE-1) break; /* leave space for \0 */
                   memmove(buf+cursor+1, buf+cursor, cmdlen-cursor);
                   buf[cursor++] = c;
                   cmdlen++;
                   if(uart_echo) {
                    uart_tx(c);
                    uart_puts(buf+cursor, cmdlen-cursor);
                    for (i=cursor; i<cmdlen; i++) uart_tx(0x08);
                   }
 }

 if(state == LNEDIT_DONE) {
  state = LNEDIT_START;

  if (cmd_hist_len == 0 || strcmp(cmd_hist + (cmd_hist_len-1)*CMD_BUF_SIZE, (char*)buf) != 0) { /* avoid saving same command twice  */
   if (cmd_hist_len == CMD_HISTORY_SIZE) {
    memmove(cmd_hist, cmd_hist+CMD_BUF_SIZE, CMD_BUF_SIZE*(CMD_HISTORY_SIZE-1));
    cmd_hist_len--;
   }
   buf[cmdlen] = 0;
   memmove(cmd_hist+cmd_hist_len*CMD_BUF_SIZE, buf, CMD_BUF_SIZE);
   ++cmd_hist_len;
  }
 }
 return cmd;
}

static uint8_t 
get_read_length(const uint8_t *buf, uint8_t len) 
{
 uint8_t l = 0;
 if(len > 0 && ishexdigit(buf[0])) {
  l = hex2dec(buf[0]);
  if(len > 1 && ishexdigit(buf[1])) {
   l <<= 4;
   l |= hex2dec(buf[1]);
  }
 }
 return l;
}

static uint8_t 
get_set_opt(const uint8_t *buf, uint8_t len)
{
 uint16_t v;
 uint8_t i, w = 0;
 uint8_t *opt_eep;
 uint8_t max;
 uint8_t *opt;

 if(len == 0) {
   printf_P(PSTR("ERROR\r\n"));
   return 0;
 }
 switch(buf[0]) {
          case 'X':
                  opt = &hp3478_ext_enable;
                  opt_eep = &hp3478_ext_en_eep;
                  max = 1;
                  break;
          case 'I':
                  opt = &uart_echo;
                  opt_eep = &uart_echo_eep;
                  max = 1;
                  break;
          case 'C':
                  opt = &gpib_my_addr;
                  opt_eep = &gpib_my_addr_eep;
                  max = 30;
                  break;
          case 'D':
                  opt = &gpib_hp3478_addr;
                  opt_eep = &gpib_hp3478_addr_eep;
                  max = 31;
                  break;
          case 'R':
                  opt = &gpib_end_seq_rx;
                  opt_eep = &gpib_end_seq_rx_eep;
                  max = 7;
                  break;
          case 'T':
                  opt = &gpib_end_seq_tx;
                  opt_eep = &gpib_end_seq_tx_eep;
                  max = 7;
                  break;
          case 'B':
                  opt = &uart_baud;
                  opt_eep = &uart_baud_eep;
                  max = 4;
                  break;
          case '0':
          case '1':
                  set_defaults(buf[0]-'0');
                  printf_P(PSTR("OK\r\n"));
                  return 1;
          case '?':
                  printf_P(opt_help);
                  return 0;
          default:
                  printf_P(PSTR("WRONG OPTION\r\n"));
                  return 0;
 }
 buf++;
 len--;

 if(len == 0) {
  printf_P(PSTR("%d\r\n"), (int)*opt);
  return 0;
 }

 for(i = 0, v = 0; i < len; i++) {
  uint8_t c = buf[i];
  
  if(c > '9' || c < '0') {
   if((c == 'w' || c == 'W') && i == len-1) {
    w = 1;
    break;
   }
   printf_P(PSTR("ERROR\r\n"));
   return 0;
  }
  v = v*10 + (c-'0');
 }
 if(v > max) {
  printf_P(PSTR("ERROR\r\n"));
  return 0;
 }

 *opt = (uint8_t)v;
 if(w) eeprom_write_byte(opt_eep, (uint8_t)v);
 printf_P(PSTR("OK\r\n"));
 return 1;
}

static void
gpib_state_from_cmd(const uint8_t *buf, uint8_t len)
{
 uint8_t i;
 uint8_t b;
                   
 for (i=0; i<len; i++) {
  b = buf[i];
                    if (b == '?' || b == 64+gpib_my_addr) {
                     gpib_state = 0;
                     led_set(LED_OFF);
                    } else if (b == 32+gpib_my_addr) {
                     gpib_state = GPIB_LISTEN;
                     led_set(LED_FAST);
                    }
 }
}

static void 
command_handler(uint8_t command, uint8_t *buf, uint8_t len)
{
  uint8_t gpib_buf[GPIB_BUF_SIZE];
  uint8_t gpib_len;
  uint8_t i;
  uint8_t send_eoi;
  uint8_t result;

   switch(command) {
           case 'D': /* send/receive ASCII */
                   if (gpib_state == GPIB_LISTEN) {
                    uart_rx_esc_char(); /* clear previous escape */
                    do {
                     result = gpib_receive(gpib_buf, GPIB_BUF_SIZE, &gpib_len, gpib_end_seq_rx);
                     uart_puts(gpib_buf, gpib_len);
                    } while(result == GPIB_END_BUF && !uart_rx_esc_char());
                    if(result == 0) printf_P(PSTR("\r\n")); /* no EOI or EOL received, ensure user receives
                                                               at least an empty line */
                    break;
                   }
                   result = gpib_transmit(buf+1, len-1, gpib_end_seq_tx); 
                   if (result == len-1) printf_P(PSTR("OK\r\n"));
                   else printf_P(PSTR("TIMEOUT %d\r\n"), (unsigned) result);
                   break;
           case 'C': /* send ASCII command */
                   gpib_state_from_cmd(buf+1, len-1); 

                   gpib_talk();

                   set_atn(1);
                   result = gpib_transmit(buf+1, len-1, 0);

                   if (result == len-1) printf_P(PSTR("OK\r\n"));
                   else printf_P(PSTR("TIMEOUT %d\r\n"), (unsigned) result);

                   set_atn(0);

                   if (gpib_state == GPIB_LISTEN) gpib_listen();

                   break;
           case 'R':
                   set_ren(1);
                   printf_P(PSTR("OK\r\n"));
                   break;
           case 'L':
                   set_ren(0);
                   printf_P(PSTR("OK\r\n"));
                   break;
           case 'I':
                   SetIFC(0);
                   _delay_ms(1);
                   SetIFC(1);
                   if (gpib_state == GPIB_LISTEN) {
                    gpib_state = 0;
                    led_set(LED_OFF);
                    gpib_talk();
                   }
                   printf_P(PSTR("OK\r\n"));
                   break;
           case 'S':
                   uart_tx(ren()?'1':'0');
                   uart_tx(srq()?'1':'0');
                   uart_tx(gpib_state+'0');
                   uart_tx(13);
                   uart_tx(10);
                   break;

           case 'P':
                   led_set(LED_SLOW);
                   gpib_listen();

                   uart_rx_esc_char();
                   while (!uart_rx_esc_char()) {
                    result = gpib_receive(gpib_buf, 1, &gpib_len, 0);
                    if(gpib_len == 0) _delay_ms(10);
                    else uart_tx(gpib_buf[0]);
                   }
                   gpib_state = 0;
                   gpib_talk();
                   led_set(LED_OFF);
                   break;
           case '?':
                   printf_P(help);
                   break;
           case 'H':
                   for (i=0; i < cmd_hist_len; i++)
                    printf_P(PSTR("%d: %s\r\n"), i, cmd_hist+i*CMD_BUF_SIZE);
                   break;
           case 'T': /* THC - transfer command hex
                        THD transfer hex data, TBD transfer binary data */

                   if(len < 3) {
                    printf_P(PSTR("ERROR\r\n"));
                    break; 
                   }
                   if(buf[1] == 'H' && (gpib_state != GPIB_LISTEN || buf[2] == 'C')) { /* HEX tx command & data */
                     if (!convert_hex_message(buf+3, len-3, gpib_buf, &gpib_len, &send_eoi)) {
                      printf_P(PSTR("ERROR\r\n"));
                      break;
                     }
                     if(buf[2] == 'C') {
                      gpib_state_from_cmd(gpib_buf, gpib_len); 
                      gpib_talk();
                      set_atn(1);
                      send_eoi = 0;
                     }
                     result = gpib_transmit(gpib_buf, gpib_len, 0);

                     if (result == gpib_len) printf_P(PSTR("OK\r\n"));
                     else printf_P(PSTR("TIMEOUT %d\r\n"), (unsigned) result);

                     if(buf[2] == 'C') {
                      set_atn(0);
                      if (gpib_state == GPIB_LISTEN) gpib_listen();
                     }
                   } else if(buf[1] == 'B' && buf[2] == 'D'&& gpib_state != GPIB_LISTEN) { /* binary tx data */
                    uint8_t err = 0;
                    while(1) {
                     gpib_len = uart_rx();
                     send_eoi = 0;
                     if(gpib_len & 0x80) {
                      gpib_len &= 0x7f;
                      send_eoi = GPIB_END_EOI;
                     }
                     if(gpib_len == 0) break;
                     for(i = 0; i < gpib_len; i++) gpib_buf[i] = uart_rx();
                     if(!err) {
                      result = gpib_transmit(gpib_buf, gpib_len, send_eoi);
                      err = result != gpib_len;
                     }
                     uart_tx(result);
                    }
                   } else if((buf[1] == 'B' || buf[1] == 'H') && buf[2] == 'D') { /* hex & binary rx data */
                    uint32_t l;
                    
                    l = get_read_length(buf+3, len-3);
                    if(l == 0) l = 0xffffffff;
                    uart_rx_esc_char(); /* clear previous escape */
                    do {
                     gpib_len = l > GPIB_BUF_SIZE ? GPIB_BUF_SIZE : l;
                     result = gpib_receive(gpib_buf, gpib_len, &gpib_len, gpib_end_seq_rx);
                     if(buf[1] == 'H') for (i=0; i<gpib_len; i++) printf_P(PSTR("%02X"), gpib_buf[i]);
                     else if(gpib_len) {
                      uart_tx(gpib_len | ((result & GPIB_END_EOI) ? 0x80 : 0));
                      uart_puts(gpib_buf, gpib_len);
                     }
                     l -= gpib_len;
                    } while(result == GPIB_END_BUF && l != 0 && !uart_rx_esc_char());
                    if(buf[1] == 'B') {
                     uart_tx(0);
                    } else {
                     if((result & GPIB_END_EOI) == 0) uart_tx(';');
                     printf_P(PSTR("\r\n"));
                    }
                   } else {
                    printf_P(PSTR("ERROR\r\n"));
                   }
                   break;
                    
           case 'O':
                   if(get_set_opt(buf+1, len-1)) {
                    if(buf[1] == 'B') {
                     while(!uart_tx_empty());
                     _delay_ms(1); /* data may remain in registers, wait for transmission
                                      caller should wait at least 2ms after OK responce before
                                      transmitting data with new BR */
                     uart_set_speed(uart_baud);
                    }
                   }

                   break;
           case 0: 
           case 13: 
                   break;
           default:
                   printf_P(PSTR("WRONG COMMAND\r\n"));
   }
}

struct hp3478_reading {
 int32_t value;
 uint8_t dot;
 int8_t exp;
};

/* flags for IO routines below */
#define HP3478_CMD_LISTEN                 1 /* stay in listen state (hp3478a is talking, we are listening) */
#define HP3478_CMD_TALK                   2 /* stay in talk state (hp3478a is listening, we are talking) */
#define HP3478_CMD_REMOTE                 4 /* leave REN active */
#define HP3478_CMD_CONT     (HP3478_CMD_REMOTE|HP3478_CMD_TALK|HP3478_CMD_LISTEN)
#define HP3478_DISP_HIDE_ANNUNCIATORS      8
#define HP3478_CMD_NO_LF                 16 /* do not send LF after the command */

uint8_t hp3478_saved_state[2];

static uint8_t 
hp3478_cmd(const uint8_t *cmd, uint8_t len, uint8_t flags)
{
 uint8_t st = gpib_state;
 uint8_t cmd1[2];

 set_ren(1);
 if(st != GPIB_TALK) {
   if(st == GPIB_LISTEN) gpib_talk();
   cmd1[0] = gpib_hp3478_addr+GPIB_LISTEN_ADDR_OFFSET;
   cmd1[1] = gpib_my_addr+GPIB_TALK_ADDR_OFFSET;
   set_atn(1);
   if(!gpib_transmit_b(cmd1, 2, 0)) goto fail;
   set_atn(0);
 }
 if(!gpib_transmit_b(cmd, len, (flags & HP3478_CMD_NO_LF) ? 0 : GPIB_END_LF)) goto fail;
 if((flags & HP3478_CMD_REMOTE) == 0) set_ren(0);
 if((flags & HP3478_CMD_TALK) == 0) {
  cmd1[0] = '?';
  set_atn(1);
  if(!gpib_transmit_b(cmd1, 1, 0)) goto fail;
  set_atn(0);
  gpib_state = 0;
 } else {
  gpib_state = GPIB_TALK;
 }

 return 1;
fail:
 set_atn(0);
 set_ren(0);
 gpib_state = 0;
 return 0;
}

static uint8_t 
hp3478_cmd_P(const char *cmd, uint8_t flags)
{
 uint8_t st = gpib_state;
 uint8_t cmd1[2];

 set_ren(1);
 if(st != GPIB_TALK) {
   if(st == GPIB_LISTEN) gpib_talk();
   cmd1[0] = gpib_hp3478_addr+GPIB_LISTEN_ADDR_OFFSET;
   cmd1[1] = gpib_my_addr+GPIB_TALK_ADDR_OFFSET;
   set_atn(1);
   if(!gpib_transmit_b(cmd1, 2, 0)) goto fail;
   set_atn(0);
 }
 if(!gpib_transmit_P((const uint8_t*)cmd, strlen_P(cmd), GPIB_END_LF)) goto fail;
 if((flags & HP3478_CMD_REMOTE) == 0) set_ren(0);
 if((flags & HP3478_CMD_TALK) == 0) {
  cmd1[0] = '?';
  set_atn(1);
  if(!gpib_transmit_b(cmd1, 1, 0)) goto fail;
  set_atn(0);
  gpib_state = 0;
 } else {
  gpib_state = GPIB_TALK;
 }

 return 1;
fail:
 set_atn(0);
 set_ren(0);
 gpib_state = 0;
 return 0;
}

static uint8_t
hp3478_get_srq_status(uint8_t *sb)
{
 uint8_t cmd[3];
 uint8_t rl;
 uint8_t st = gpib_state;

 if(st == GPIB_LISTEN) gpib_talk();
 gpib_state = 0;
 cmd[0] = 24; /* serial poll enable */
 cmd[1] = gpib_hp3478_addr+GPIB_TALK_ADDR_OFFSET;
 cmd[2] = gpib_my_addr+GPIB_LISTEN_ADDR_OFFSET;
 set_atn(1);
 if(!gpib_transmit_b(cmd, 3, 0)) goto fail;
 set_atn(0);
 gpib_listen();
 gpib_receive(sb, 1, &rl, 0);
 if(rl != 1) goto fail;
 gpib_talk();
 set_atn(1);
 cmd[0] = 25; /* serial poll disable */
 cmd[1] = '_';
 if(!gpib_transmit_b(cmd, 2, 0)) goto fail;
 set_atn(0);
 return 1;
fail:
 gpib_talk();
 set_atn(0);
 return 0;
}

static uint8_t
hp3478_read(uint8_t *buf, uint8_t buf_sz, uint8_t *rl, uint8_t flags)
{
 uint8_t cmd[2];

 if(gpib_state != GPIB_LISTEN) {
  cmd[0] = gpib_my_addr+GPIB_LISTEN_ADDR_OFFSET;
  cmd[1] = gpib_hp3478_addr+GPIB_TALK_ADDR_OFFSET;
  set_atn(1);
  if(!gpib_transmit_b(cmd, 2, 0)) {
   printf_P(PSTR("read: can't listen\r\n"));
   goto fail;
  }
  set_atn(0);
  gpib_listen();
 }
 if(gpib_receive(buf, buf_sz, rl, GPIB_END_EOI) != GPIB_END_EOI) {
  printf_P(PSTR("read: can't receive\r\n"));
  goto fail;
 }
 if((flags & HP3478_CMD_LISTEN) == 0) {
  gpib_talk();
  set_atn(1);
  cmd[0] = '_';
  if(!gpib_transmit_b(cmd, 1, 0)) {
  printf_P(PSTR("read: can't untalk\r\n"));
   goto fail;
  }
  set_atn(0);
  gpib_state = 0;
 } else {
  gpib_state = GPIB_LISTEN;
 }
 return 1;
fail:
 gpib_talk();
 set_atn(0);
 gpib_state = 0;
 return 0;
}

static uint8_t
hp3478_display(const char display[], uint8_t len, uint8_t flags)
{
 uint8_t cmd[2];

 cmd[0] = 'D'; /* display */
 cmd[1] = (flags & HP3478_DISP_HIDE_ANNUNCIATORS) != 0 ? '3' : '2';
 if(!hp3478_cmd(cmd, 2, HP3478_CMD_CONT|HP3478_CMD_NO_LF)) return 0;
 if(!hp3478_cmd((const uint8_t*)display, len, HP3478_CMD_CONT)) return 0;
 if(!hp3478_cmd(0, 0, flags)) return 0; /* send additional LF */
 return 1;
}

static uint8_t
hp3478_display_P(const char *display, uint8_t flags)
{
 uint8_t cmd[2];

 cmd[0] = 'D'; /* display */
 cmd[1] = (flags & HP3478_DISP_HIDE_ANNUNCIATORS) != 0 ? '3' : '2';
 if(!hp3478_cmd(cmd, 2, HP3478_CMD_CONT|HP3478_CMD_NO_LF)) return 0;
 if(!hp3478_cmd_P(display, HP3478_CMD_CONT)) return 0;
 if(!hp3478_cmd(0, 0, flags)) return 0; /* send additional LF */
 return 1;
}

static uint8_t 
hp3478_get_reading(struct hp3478_reading *reading, uint8_t flags)
{
 uint8_t buf[13];
 uint8_t sign;
 int32_t v;
 uint8_t i;
 uint8_t len;

 if(!hp3478_read(buf, sizeof(buf), &len, flags)) return 0;

 i = 0;
 sign = buf[0] == '-';
 v = 0;
 for(++i; i < len; i++) {
  if(buf[i] == 'E') break;
  if(buf[i] == '.') reading->dot = i-1;
  else v = v*10 + (buf[i]-'0');
 }
 i++;
 if(len-i < 2) return 0;
 reading->value = sign ? -v : v;
 sign = buf[i++] == '-';
 reading->exp = sign ? '0'-buf[i] : buf[i]-'0';
 return 1;
}

static uint8_t
hp3478_get_status(uint8_t st[5])
{
 uint8_t rl;
 if(!hp3478_cmd_P(PSTR("B"), HP3478_CMD_TALK)) return 0;
 if(!hp3478_read(st, 5, &rl, 0)) return 0;
 return rl == 5;
}

static uint8_t
hp3478_display_reading(struct hp3478_reading *r, uint8_t st, char mode_ind, uint8_t flags)
{
 char display[13];
 char exp_char;
 const char *m;
 int8_t i;
 uint8_t f;

 f = st & HP3478_ST_FUNC;
 if(r->exp == 9 && r->value >= 999900) {
  uint8_t dot;

  switch(st&(HP3478_ST_RANGE|HP3478_ST_FUNC)) {
          case HP3478_ST_RANGE2|HP3478_ST_FUNC_DCA:
          case HP3478_ST_RANGE2|HP3478_ST_FUNC_ACA:
          case HP3478_ST_RANGE3|HP3478_ST_FUNC_DCV:
          case HP3478_ST_RANGE3|HP3478_ST_FUNC_2WOHM:
          case HP3478_ST_RANGE3|HP3478_ST_FUNC_4WOHM:
          case HP3478_ST_RANGE6|HP3478_ST_FUNC_2WOHM:
          case HP3478_ST_RANGE6|HP3478_ST_FUNC_4WOHM:
                  dot = 1;
                  break;
          case HP3478_ST_RANGE1|HP3478_ST_FUNC_DCV:
          case HP3478_ST_RANGE1|HP3478_ST_FUNC_2WOHM:
          case HP3478_ST_RANGE1|HP3478_ST_FUNC_4WOHM:
          case HP3478_ST_RANGE3|HP3478_ST_FUNC_ACV:
          case HP3478_ST_RANGE4|HP3478_ST_FUNC_DCV:
          case HP3478_ST_RANGE4|HP3478_ST_FUNC_2WOHM:
          case HP3478_ST_RANGE4|HP3478_ST_FUNC_4WOHM:
          case HP3478_ST_RANGE7|HP3478_ST_FUNC_2WOHM:
          case HP3478_ST_RANGE7|HP3478_ST_FUNC_4WOHM:
                  dot = 2;
                  break;
          default:
                  dot = 3;

  }
  display[0] = ' ';
  display[1] = ' ';
  i = 2;
 /* FIXME: dot is probably incorrect */
  if(dot == 1) display[i++] = '.';
  display[i++] = 'O';
  if(dot == 2) display[i++] = '.';
  display[i++] = 'V';
  if(dot == 3) display[i++] = '.';
  display[i++] = 'L';
  display[i++] = 'D';
  while(i != 8) display[i++] = ' ';
  switch(st&(HP3478_ST_RANGE|HP3478_ST_FUNC)) {
          case HP3478_ST_RANGE1|HP3478_ST_FUNC_DCV:
          case HP3478_ST_RANGE1|HP3478_ST_FUNC_ACV:
          case HP3478_ST_RANGE1|HP3478_ST_FUNC_DCA:
          case HP3478_ST_RANGE1|HP3478_ST_FUNC_ACA:
          case HP3478_ST_RANGE2|HP3478_ST_FUNC_DCV:
          case HP3478_ST_RANGE6|HP3478_ST_FUNC_2WOHM:
          case HP3478_ST_RANGE6|HP3478_ST_FUNC_4WOHM:
          case HP3478_ST_RANGE7|HP3478_ST_FUNC_2WOHM:
          case HP3478_ST_RANGE7|HP3478_ST_FUNC_4WOHM:
                  exp_char = 'M';
                  break;
          case HP3478_ST_RANGE3|HP3478_ST_FUNC_2WOHM:
          case HP3478_ST_RANGE3|HP3478_ST_FUNC_4WOHM:
          case HP3478_ST_RANGE4|HP3478_ST_FUNC_2WOHM:
          case HP3478_ST_RANGE4|HP3478_ST_FUNC_4WOHM:
          case HP3478_ST_RANGE5|HP3478_ST_FUNC_2WOHM:
          case HP3478_ST_RANGE5|HP3478_ST_FUNC_4WOHM:
                  exp_char = 'K';
                  break;
          default:
                  exp_char = ' ';

  }
  goto display_units;
 }
 if(r->value >= 0) { 
  if(f == HP3478_ST_FUNC_DCA || f == HP3478_ST_FUNC_DCV) display[0] = '+';
  else display[0] = ' ';
 } else {
  display[0] = '-';
  r->value = -r->value;
 }
  
 for(i = 7; i > 0; i--) {
  if(((st & HP3478_ST_N_DIGITS) != HP3478_ST_N_DIGITS5 && i == 7)
    || ((st & HP3478_ST_N_DIGITS) == HP3478_ST_N_DIGITS3 && i == 6)) display[i] = ' ';
  else display[i] = r->value % 10 + '0';
  r->value /= 10;
  if(i == r->dot+2) display[--i] = '.';
 }

 switch(r->exp) {
   case -3: exp_char = 'M'; break;
   case 0: exp_char = ' '; break;
   case 3: exp_char = 'K'; break;
   case 6: exp_char = 'M'; break;
   case 9: exp_char = 'G'; break;
   default: exp_char = '?';
 }

display_units:
 i = 8;
 if(!mode_ind) display[i++] = ' ';
 display[i++] = exp_char;
 if(mode_ind == 'd') m = PSTR("V  ");
 else if(mode_ind == 'c') m = PSTR("C  ");
 else switch(f) {
         case HP3478_ST_FUNC_DCV: m = PSTR("VDC"); break;
         case HP3478_ST_FUNC_ACV: m = PSTR("VAC"); break;
         case HP3478_ST_FUNC_2WOHM:
         case HP3478_ST_FUNC_4WOHM: m = PSTR("OHM"); break;
         case HP3478_ST_FUNC_DCA: m = PSTR("ADC"); break;
         case HP3478_ST_FUNC_ACA: m = PSTR("ACA"); break;
         default: m = PSTR("???"); 
 }
 memcpy_P(display+i, m, 3);
 if(mode_ind) display[12] = mode_ind < 'a' ? mode_ind : ' ';
#if 0
 {
  static uint8_t a = 0;
  a++;
  if(a != 20) return 1;
  a = 0;
 }
#endif
 return hp3478_display(display, sizeof(display), flags );
}

static uint8_t hp3478_rel_mode;
static struct hp3478_reading hp3478_rel_ref;
static uint8_t
hp3478_rel_start(uint8_t st1, const struct hp3478_reading *r)
{
 if(!hp3478_cmd_P(PSTR("M21T1"), 0)) return 0;
 hp3478_rel_mode = st1;
 hp3478_rel_ref = *r;
 return 1;
}

static uint8_t
hp3478_rel_handle_data(struct hp3478_reading *r)
{
 struct hp3478_reading ref, in, out;
 int8_t e_ref, e_in;
 int8_t i;

 ref = hp3478_rel_ref;
 in = *r;
 e_ref = ref.exp + ref.dot;
 e_in = in.exp + in.dot;

 if(e_in >= e_ref) {
  for(i = e_ref; i < e_in; i++) ref.value /= 10;
  out.dot = in.dot;
  out.exp = in.exp;
 } else {
  for(i = e_in; i < e_ref; i++) in.value /= 10;
  out.dot = ref.dot;
  out.exp = ref.exp;
 }
 out.value = in.value - ref.value;

 return hp3478_display_reading(&out, hp3478_rel_mode, '*', 0);
}

static uint8_t hp3478_menu_pos;
#define HP3478_MENU_ERROR   1
#define HP3478_MENU_DONE    2
#define HP3478_MENU_NOP     3
#define HP3478_MENU_WAIT    4
#define HP3478_MENU_XOHM    5
#define HP3478_MENU_BEEP    6 /* continuity test */
#define HP3478_MENU_XOHM_BEEP   7 /* continuity test, different menu path */
#define HP3478_MENU_MINMAX  8
#define HP3478_MENU_AUTOHOLD 9
#define HP3478_MENU_OHM_MINMAX  10
#define HP3478_MENU_OHM_AUTOHOLD 11
#define HP3478_MENU_TEMP 12
#define HP3478_MENU_DIODE 13

static uint8_t hp3478_btn_detect_stage;

static uint8_t 
hp3478_menu_next(uint8_t st1, struct hp3478_reading *r, uint8_t pos)
{
 switch(pos) {
         case 0:
                 if((st1 & HP3478_ST_FUNC) == HP3478_ST_FUNC_2WOHM) {
                  if(r->exp == 9) return HP3478_MENU_XOHM_BEEP;
                  return HP3478_MENU_BEEP;
                 } 
                 if((st1 & HP3478_ST_FUNC) == HP3478_ST_FUNC_XOHM) 
                  return HP3478_MENU_XOHM;
                 return HP3478_MENU_AUTOHOLD;
         case HP3478_MENU_XOHM_BEEP:
                 return HP3478_MENU_XOHM;
         case HP3478_MENU_XOHM:
         case HP3478_MENU_BEEP:
                 return HP3478_MENU_DIODE;
         case HP3478_MENU_DIODE:
                 return HP3478_MENU_OHM_AUTOHOLD;
         case HP3478_MENU_OHM_AUTOHOLD:
                 return HP3478_MENU_OHM_MINMAX;
         case HP3478_MENU_OHM_MINMAX:
                 return HP3478_MENU_TEMP;
         case HP3478_MENU_AUTOHOLD:
                 return HP3478_MENU_MINMAX;
         case HP3478_MENU_TEMP:
         case HP3478_MENU_MINMAX:
                 return HP3478_MENU_DONE;
 }
}

static uint8_t
hp3478_menu_show(uint8_t pos)
{
 const char *s;
 switch(pos) {
         case HP3478_MENU_OHM_MINMAX: s = PSTR("M: MINMAX"); break;
         case HP3478_MENU_MINMAX: s = PSTR("M: MINMAX"); break;
         case HP3478_MENU_XOHM_BEEP:
         case HP3478_MENU_BEEP: s = PSTR("M: CONT"); break;
         case HP3478_MENU_XOHM: s = PSTR("M: XOHM"); break;
         case HP3478_MENU_OHM_AUTOHOLD:
         case HP3478_MENU_AUTOHOLD: s = PSTR("M: AUTOHOLD"); break;
         case HP3478_MENU_DIODE: s = PSTR("M: DIODE"); break;
         case HP3478_MENU_TEMP: s = PSTR("M: TEMP"); break;
 }
 return hp3478_display_P(s, HP3478_DISP_HIDE_ANNUNCIATORS|HP3478_CMD_CONT);
}

static uint8_t 
hp3478_menu_restart_btn_detect(void)
{
 /* invalid command to trigger HP3478_SB_SYNERR */
 if(!hp3478_cmd_P(PSTR("A"), HP3478_CMD_REMOTE|HP3478_CMD_TALK)) return 0;
 hp3478_btn_detect_stage = 0;
 return 1;
}

static uint8_t 
hp3478_menu_init(uint8_t st1, struct hp3478_reading *r)
{
 hp3478_menu_pos = hp3478_menu_next(st1, r, 0);
 if(!hp3478_menu_show(hp3478_menu_pos)) return 0;
 if(!hp3478_menu_restart_btn_detect()) return 0;
 return 1;
}

static uint8_t 
hp3478_menu_process(uint8_t ev)
{  
 uint8_t sb;

 switch(hp3478_btn_detect_stage) {
         case 0:
                 if((ev & (EV_TIMEOUT|EV_SRQ)) != 0 && srq()) break;
                 if((ev & EV_TIMEOUT) != 0) {
                  hp3478_btn_detect_stage = 1;
                  if(!hp3478_cmd_P(PSTR("M24"), HP3478_CMD_REMOTE|HP3478_CMD_TALK)) 
                   return HP3478_MENU_ERROR;
                  return HP3478_MENU_WAIT;
                 }
                 return HP3478_MENU_NOP;
         case 1:
                 if((ev & (EV_TIMEOUT|EV_SRQ)) != 0 && !srq()) break;
                 if((ev & EV_TIMEOUT) != 0) {
                  hp3478_btn_detect_stage = 0;
                  if(!hp3478_cmd_P(PSTR("M20"), HP3478_CMD_REMOTE|HP3478_CMD_TALK)) 
                   return HP3478_MENU_ERROR;
                  return HP3478_MENU_WAIT;
                 }
                 return HP3478_MENU_NOP;
 }
 if(!hp3478_get_srq_status(&sb)) return HP3478_MENU_ERROR;
 if(!hp3478_cmd_P(PSTR("KM20"), 0)) return HP3478_MENU_ERROR;
 if(sb & HP3478_SB_FRPSRQ) {
  hp3478_menu_pos = hp3478_menu_next(0, 0, hp3478_menu_pos);
  if(hp3478_menu_pos == HP3478_MENU_DONE) return HP3478_MENU_DONE;
  if(!hp3478_menu_show(hp3478_menu_pos)) return HP3478_MENU_ERROR;
 } else {
  /* assume Local button pressed */
  return hp3478_menu_pos;
 }
                  
 hp3478_menu_restart_btn_detect();
 return HP3478_MENU_WAIT;
}

static uint32_t hp3478_xohm_10M;

static uint8_t 
hp3478_xohm_init(void)
{
 hp3478_xohm_10M = 0;
 if(!hp3478_cmd_P(PSTR("F7M21"), 0)) return 0; // FIXME: F7N5M21?
 return 1;
}

static uint8_t 
hp3478_xohm_handle_data(struct hp3478_reading *reading)
{
 uint32_t r, n;
 struct hp3478_reading rr = *reading;

 if(hp3478_xohm_10M == 0) hp3478_xohm_10M = rr.value;

 if(hp3478_xohm_10M <= rr.value + 100)  {
  if(!hp3478_display_P(PSTR("  OVLD  GOHM"), 0)) return 0;
  return 1;
 }
 if(rr.value < 0) rr.value = 0;
 r = (uint64_t)hp3478_xohm_10M * rr.value / (hp3478_xohm_10M-rr.value);
 rr.exp = 6;
 rr.dot = 2;
 for(n = 1000000; r > n; ) {
  rr.dot++;
  if(rr.dot == 4) {
   rr.exp += 3;
   rr.dot = 1;
  }
  r /= 10;
 }
 rr.value = r;
 return hp3478_display_reading(&rr, HP3478_ST_FUNC_2WOHM|HP3478_ST_N_DIGITS5, 0, 0);
}

static uint8_t minmax_state = 0;
static uint8_t 
hp3478_diode_init(void)
{
 uint8_t s[5];
 if(!hp3478_get_status(s)) return 0;
 hp3478_saved_state[0] = s[0];
 hp3478_saved_state[1] = s[1];
 if(!hp3478_cmd_P(PSTR("R3M21"), 0)) return 0;
 minmax_state = 1;
 return 1;
}

static uint8_t 
hp3478_diode_handle_data(struct hp3478_reading *reading)
{
 if(reading->exp == 9) {
  if(minmax_state) {
   minmax_state = 0;
   if(!hp3478_display_P(PSTR("     >3 V"), HP3478_DISP_HIDE_ANNUNCIATORS)) return 0;
  }
  return 1;
 }
 minmax_state = 1;
 reading->exp = 0;
 return hp3478_display_reading(reading, hp3478_saved_state[0], 'd', 0);
}

static uint8_t 
hp3478_temp_init(void)
{
 uint8_t s[5];
 if(!hp3478_get_status(s)) return 0;
 hp3478_saved_state[0] = s[0];
 /* hp3478_saved_state[1] = s[1]; */
 if(!hp3478_cmd_P(PSTR("M21"), 0)) return 0;
 minmax_state = 1;
 return 1;
}

static uint8_t 
hp3478_temp_handle_data(struct hp3478_reading *reading)
{
 if(reading->exp == 9) {
  if(minmax_state) {
   minmax_state = 0;
   if(!hp3478_display_P(PSTR("  OPEN"), HP3478_DISP_HIDE_ANNUNCIATORS)) return 0;
  }
  return 1;
 }
 minmax_state = 1;
#define RTD_A 3.908e-3
#define RTD_B -5.8019e-7
#define RTD_C -4.2735e-12
#define RTD_R0 1000.0
 { 
  uint8_t i;
  double t, r = reading->value;
  //printf_P(PSTR("temp: %u->%lu\r\n"), (unsigned)6-reading->dot-reading->exp, reading->value);
  for(i = 6-reading->dot-reading->exp; i != 0; i--) r /= 10;
  t = (-(RTD_R0*RTD_A)+sqrt((RTD_R0*RTD_R0*RTD_A*RTD_A) - (4*RTD_R0*RTD_B)*(RTD_R0-r)))/(2*RTD_R0*RTD_B);
  reading->value = t*1000;
  //printf_P(PSTR("temp: %lu\r\n"), (uint32_t)t);
  reading->exp = 0;
  reading->dot = 3;
 }
 return hp3478_display_reading(reading, hp3478_saved_state[0], 'c', 0);
}


static uint8_t 
hp3478_cont_fini(void)
{
 uint8_t s1, s2;
 uint8_t cmd[6];

 s1 = hp3478_saved_state[0];
 s2 = hp3478_saved_state[1];
 beep(0);
 cmd[0] = 'R';
 if(s2 & HP3478_ST_AUTORANGE) cmd[1] = 'A';
 else cmd[1] = '0' + ((s1&HP3478_ST_RANGE)>>2);
 cmd[2] = 'N';
 switch(s1 & HP3478_ST_N_DIGITS) {
         case HP3478_ST_N_DIGITS5: cmd[3] = '5'; break;
         case HP3478_ST_N_DIGITS4: cmd[3] = '4'; break;
         case HP3478_ST_N_DIGITS3: cmd[3] = '3'; break;
 }
 cmd[4] = 'Z';
 if(s2 & HP3478_ST_AUTOZERO) cmd[5] = '1';
 else cmd[5] = '0';
 return hp3478_cmd(cmd, sizeof(cmd), 0);
}

static uint8_t 
hp3478_cont_init(void)
{
 uint8_t s[5];
 if(!hp3478_get_status(s)) return 0;
 hp3478_saved_state[0] = s[0];
 hp3478_saved_state[1] = s[1];
 if(!hp3478_cmd_P(PSTR("R2N3M21Z0"), 0)) return 0;
 if(!hp3478_display_P(PSTR(" >100 OHM"), HP3478_DISP_HIDE_ANNUNCIATORS)) return 0;
 return 1;
}

#define MINMAX_MIN       1
#define MINMAX_MAX       2
#define MINMAX_DISP     12
#define MINMAX_DISP_NONE 0
#define MINMAX_DISP_MIN  4
#define MINMAX_DISP_MAX  8
static struct hp3478_reading minmax_min;
static struct hp3478_reading minmax_max;
static uint8_t 
hp3478_minmax_init(void)
{
 uint8_t s[5];
 if(!hp3478_get_status(s)) return 0;
 hp3478_saved_state[0] = s[0];
 if(!hp3478_cmd_P(PSTR("M21"), 0)) return 0;
 minmax_state = 0;
 return 1;
}

static int8_t 
hp3478_cmp_readings(const struct hp3478_reading *r1, const struct hp3478_reading *r2)
{
 int32_t rr1 = r1->value;
 int32_t rr2 = r2->value;
 int8_t e1 = r1->exp+r1->dot;
 int8_t e2 = r2->exp+r2->dot;

 if(rr1 < 0 && rr2 >= 0) return -1;
 if(rr2 >= 0 && rr1 < 0) return 1;
 if(e1 >= e2) 
  while(1) {
    if(rr1 > rr2) return 1;
    if(e1 == e2) {
     if(rr1 == rr2) return 0;
     return -1;
    }
    rr1 *= 10;
    e1--;
  }
 while(1) {
    if(rr2 > rr1) return -1;
    if(e1 == e2) {
     if(rr1 == rr2) return 0;
     return 1;
    }
    rr2 *= 10;
    e2--;
 }
}

static uint8_t 
hp3478_minmax_detect_key(void)
{
 if(!srq()) return 0;
 if(!hp3478_cmd_P(PSTR("M20"), HP3478_CMD_CONT)) {
  printf_P(PSTR("M20 failed\r\n"));
  return 1;
 }
 _delay_us(400); /* ~250 uS it takes to clear SRQ after mask update */
 if(srq()) return 1;
 return 0;
}

static uint8_t 
hp3478_minmax_handle_data(struct hp3478_reading *reading)
{
 uint8_t s, r;
 s = minmax_state;

 r = 0;
 if(reading->exp != 9) {
  if((s & MINMAX_MIN) == 0 || hp3478_cmp_readings(reading, &minmax_min) < 0) {
   minmax_min = *reading;
   r |= MINMAX_MIN;
  }
  if((s & MINMAX_MAX) == 0 || hp3478_cmp_readings(reading, &minmax_max) > 0) {
   minmax_max = *reading;
   r |= MINMAX_MAX;
  }
 }
 minmax_state = s | r;
 return r;
}

static uint8_t 
hp3478_minmax_display_data(uint8_t r, uint8_t key_press)
{
 uint8_t s = minmax_state;
 struct hp3478_reading d;

 switch(s & MINMAX_DISP) {
         case MINMAX_DISP_NONE:
                 if(!key_press) break;
                 minmax_state = (s & ~MINMAX_DISP) | MINMAX_DISP_MIN;
                 if((s & MINMAX_MIN) == 0) {
                  if(!hp3478_display_P(PSTR("NO MIN"), HP3478_CMD_CONT|HP3478_DISP_HIDE_ANNUNCIATORS)) return 0;
                 } else {
                  d = minmax_min;
                  if(!hp3478_display_reading(&d, hp3478_saved_state[0], '-', HP3478_CMD_CONT|HP3478_DISP_HIDE_ANNUNCIATORS)) return 0;
                 }
                 return 1;
         case MINMAX_DISP_MIN:
                 if(!key_press) {
                  if((r & MINMAX_MIN) == 0) break;
                  d = minmax_min;
                  if(!hp3478_display_reading(&d, hp3478_saved_state[0], '-', HP3478_CMD_CONT|HP3478_DISP_HIDE_ANNUNCIATORS)) return 0;
                  return 1;
                 }
                 minmax_state = (s & ~MINMAX_DISP) | MINMAX_DISP_MAX;
                 if((s & MINMAX_MAX) == 0) {
                  if(!hp3478_display_P(PSTR("NO MAX"), HP3478_CMD_CONT|HP3478_DISP_HIDE_ANNUNCIATORS)) return 0;
                 } else {
                  d = minmax_max;
                  if(!hp3478_display_reading(&d, hp3478_saved_state[0], '+', HP3478_CMD_CONT|HP3478_DISP_HIDE_ANNUNCIATORS)) return 0;
                 }
                 return 1;
         case MINMAX_DISP_MAX:
                 if(!key_press) {
                  if((r & MINMAX_MAX) == 0) break;
                  d = minmax_max;
                  if(!hp3478_display_reading(&d, hp3478_saved_state[0], '+', HP3478_CMD_CONT|HP3478_DISP_HIDE_ANNUNCIATORS)) return 0;
                  return 1;
                 }
                 minmax_state = s & ~MINMAX_DISP;
                 if(!hp3478_cmd_P("D1", HP3478_CMD_CONT)) return 0;
                 return 1;
 }
 /* nothing to do */
 return 1;
}

static uint8_t ahld_n_stable;
static uint8_t
hp3478_autohold_init(void)
{
 uint8_t s[5];
 ahld_n_stable = 0;
 if(!hp3478_get_status(s)) return 0;
 hp3478_saved_state[0] = s[0];
 hp3478_saved_state[1] = s[1];
 if(!hp3478_cmd_P(PSTR("M21T1"), 0)) return 0;
 return 1;
}

#define HP3478_AUTOHOLD_STABLE_N 5
#define HP3478_AUTOHOLD_STABLE_D 3
static int32_t
hp3478_autohold_min_value(uint8_t st)
{
 if((st & HP3478_ST_FUNC) == HP3478_ST_FUNC_DCV &&
    (st & HP3478_ST_RANGE) <= HP3478_ST_RANGE3) return 0;
 switch(st&(HP3478_ST_FUNC|HP3478_ST_N_DIGITS)) {
         case HP3478_ST_FUNC_DCV|HP3478_ST_N_DIGITS5:
         case HP3478_ST_FUNC_ACV|HP3478_ST_N_DIGITS5:
         case HP3478_ST_FUNC_DCA|HP3478_ST_N_DIGITS5:
         case HP3478_ST_FUNC_ACA|HP3478_ST_N_DIGITS5:
                 return 10;
         case HP3478_ST_FUNC_DCV|HP3478_ST_N_DIGITS4:
         case HP3478_ST_FUNC_ACV|HP3478_ST_N_DIGITS4:
         case HP3478_ST_FUNC_DCA|HP3478_ST_N_DIGITS4:
         case HP3478_ST_FUNC_ACA|HP3478_ST_N_DIGITS4:
                 return 100;
         case HP3478_ST_FUNC_DCV|HP3478_ST_N_DIGITS3:
         case HP3478_ST_FUNC_ACV|HP3478_ST_N_DIGITS3:
         case HP3478_ST_FUNC_DCA|HP3478_ST_N_DIGITS3:
         case HP3478_ST_FUNC_ACA|HP3478_ST_N_DIGITS3:
                 return 1000;

         default:
                 return 0;

 }
}


#define AHLD_NOP      0
#define AHLD_LOCK     2
#define AHLD_UNLOCK   3
#define AHLD_ERROR    4
static uint8_t
hp3478_autohold_process(uint8_t locked, uint8_t sb)
{
 struct hp3478_reading r;
 uint8_t nstab;
 uint8_t st;
 uint8_t ret;

 if(!(sb & HP3478_SB_DREADY)) return AHLD_NOP;
 if(!hp3478_get_reading(&r, HP3478_CMD_CONT)) return AHLD_ERROR;

 nstab = ahld_n_stable;
 ret = AHLD_NOP;
 st = hp3478_saved_state[0];

 if(r.exp != minmax_min.exp || r.dot != minmax_min.dot || r.exp == 9) {
  uint8_t s[5];
  uint8_t m;
  uint8_t st1 = hp3478_saved_state[1];

  if(!hp3478_get_status(s)) return AHLD_ERROR;
  m = HP3478_ST_FUNC|HP3478_ST_N_DIGITS;
  if((st1 & HP3478_ST_AUTORANGE) == 0) m |= HP3478_ST_RANGE;
  if(((s[0] ^ st) & m) != 0
     || ((s[1] ^ st1) & HP3478_ST_AUTORANGE) != 0) {
    if(locked) {
     ret = AHLD_UNLOCK;
     locked = 0;
    }
    hp3478_saved_state[1] = s[1];
  }
  hp3478_saved_state[0] = s[0];
  st = s[0];
 } else if(nstab != 0 
           && abs(r.value - minmax_min.value) < HP3478_AUTOHOLD_STABLE_D 
           && abs(r.value) >= hp3478_autohold_min_value(st)) {
  if(++nstab == HP3478_AUTOHOLD_STABLE_N) {
#ifdef AUTO_HOLD_NO_RESUME
   if(!hp3478_cmd_P(PSTR("T4"), 0)) return AHLD_ERROR;
#else
   if(locked 
          && abs(r.value - minmax_max.value) < HP3478_AUTOHOLD_STABLE_D
          && r.exp == minmax_max.exp 
          && r.dot == minmax_max.dot) {
    ahld_n_stable = 0;
    return AHLD_NOP; /* don't beep, stable value didn't change */
   }
#endif
   minmax_max = minmax_min;
   ahld_n_stable = 0;
   if(!hp3478_display_reading(&minmax_min, st, '=', 0)) return AHLD_ERROR;
   return AHLD_LOCK;
  }
  ahld_n_stable = nstab;
  return AHLD_NOP;
 }

 minmax_min = r;
 ahld_n_stable = 1;

#ifdef AUTO_HOLD_NO_RESUME
 if(locked) ret = AHLD_UNLOCK;
#else
 if(locked) return ret;
#endif

 if(!hp3478_display_reading(&r, st, '?', 0)) return AHLD_ERROR;
 return ret;
}

#define HP3478_REINIT do { \
 state = HP3478_INIT; \
 return 250; \
} while(0)

static uint16_t
hp3478a_handler(uint8_t ev)
{
#define HP3478_DISA    0
#define HP3478_INIT    1
#define HP3478_IDLE    2
#define HP3478_RELS    3
#define HP3478_RELA    4
#define HP3478_MENU    5
#define HP3478_XOHM    6
#define HP3478_CONT    8
#define HP3478_MMAX    9
#define HP3478_AHLD   10 
#define HP3478_AHLL   11 
#define HP3478_DIOD   12 
#define HP3478_TEMP   13 
 static uint8_t state = HP3478_INIT;
 uint8_t sb;
 uint8_t st[5];
 struct hp3478_reading reading;

 if(state == HP3478_DISA) {
  if((ev & EV_EXT_ENABLE) == 0) return TIMEOUT_INF;
  state = HP3478_INIT;
 }

 if((ev & EV_EXT_DISABLE) != 0) {
  switch(state) {
          case HP3478_AHLL:
          case HP3478_AHLD:
                  beep(0);
                  hp3478_cmd_P(PSTR("M00D1T1"), 0);
                  break;
         
          case HP3478_DIOD:
          case HP3478_CONT:
                  hp3478_cont_fini();
          default:
                  hp3478_cmd_P(PSTR("M00D1"), 0);
  }
                  
  state = HP3478_DISA;
  return TIMEOUT_INF;
 }

 if(state != HP3478_INIT && state != HP3478_MENU && state != HP3478_MMAX) {
  if(!hp3478_get_srq_status(&sb)) HP3478_REINIT;
  if(sb & HP3478_SB_PWRSRQ) HP3478_REINIT;
  if(sb & HP3478_SB_FRPSRQ) {
   switch(state) {
          case HP3478_AHLL:
          case HP3478_AHLD:
                  beep(0);
                  hp3478_cmd_P(PSTR("KM20D1T1"), 0);
                  break;

          case HP3478_IDLE:
                 if((sb & HP3478_SB_DREADY) != 0)
                  if(!hp3478_get_reading(&reading, HP3478_CMD_LISTEN)) {
                   printf_P(PSTR("idle: get reading failed\r\n"));
                   HP3478_REINIT;
                  }

                 /* K is required, because serial poll doesn't clear status bits
                    immediately. Otherwise next SRQ still may be seen as FP SRQ. */  
                 if(!hp3478_cmd_P(PSTR("K"), HP3478_CMD_CONT)) HP3478_REINIT;
                 if(!hp3478_get_status(st)) HP3478_REINIT;
                 if((st[1] & HP3478_ST_INT_TRIGGER) == 0) {
                  if((sb & HP3478_SB_DREADY) == 0) {
                   if(!hp3478_cmd_P(PSTR("M21"), 0)) HP3478_REINIT;
                   state = HP3478_RELS;
                   return 1800;
                  }
                  if(reading.exp == 9) {
                   if(!hp3478_autohold_init()) HP3478_REINIT;
                   state = HP3478_AHLD;
                   return 0xffff;
                  }
                          
                  if(!hp3478_rel_start(st[0], &reading)) HP3478_REINIT;
                  state = HP3478_RELA;
                  return 0xffff;
                 }

                 if(!hp3478_menu_init(st[0], &reading)) {
                  printf_P(PSTR("idle: menu init failed\r\n"));
                  HP3478_REINIT;
                 }
                 state = HP3478_MENU;
                 return 100;

          case HP3478_CONT:
          case HP3478_DIOD:
                  hp3478_cont_fini();

          default:
                  hp3478_cmd_P(PSTR("KM20D1"), 0);
   }
   state = HP3478_IDLE;
   return TIMEOUT_INF;
  }
 }

 switch(state) {
         case HP3478_INIT:
                 if(hp3478_cmd_P(PSTR("KM20"), 0)) {
                  printf_P(PSTR("init: ok\r\n"));
                  state = HP3478_IDLE;
                  return 0xffff;
                 }
                 return 2000; /* retry initialization after 2 sec */

         case HP3478_IDLE:
                 if(!hp3478_cmd_P(PSTR("K"), 0)) HP3478_REINIT;
                 printf_P(PSTR("idle: unexpected ev %x %x\r\n"), (unsigned)ev, (unsigned)sb);
                 return TIMEOUT_INF;

         case HP3478_MENU:
                 switch(hp3478_menu_process(ev)) {
                         default:
                                                 printf_P(PSTR("menu: unknown\r\n"));
                         case HP3478_MENU_ERROR: 
                                                 printf_P(PSTR("menu: error\r\n"));
                                                 HP3478_REINIT;
                         case HP3478_MENU_BEEP: 
                         case HP3478_MENU_XOHM_BEEP: 
                                                 state = HP3478_CONT;
                                                 if(!hp3478_cont_init()) HP3478_REINIT;
                                                 return 0xffff;
                         case HP3478_MENU_XOHM: 
                                                 state = HP3478_XOHM;
                                                 printf_P(PSTR("menu: xohm\r\n"));
                                                 if(!hp3478_xohm_init()) HP3478_REINIT;
                                                 return 0xffff;
                         case HP3478_MENU_MINMAX: 
                         case HP3478_MENU_OHM_MINMAX: 
                                                 state = HP3478_MMAX;
                                                 printf_P(PSTR("menu: minmax\r\n"));
                                                 if(!hp3478_minmax_init()) HP3478_REINIT;
                                                 return 0xffff;
                         case HP3478_MENU_AUTOHOLD: 
                         case HP3478_MENU_OHM_AUTOHOLD: 
                                                 state = HP3478_AHLD;
                                                 printf_P(PSTR("menu: autohold\r\n"));
                                                 if(!hp3478_autohold_init()) HP3478_REINIT;
                                                 return 0xffff;

                         case HP3478_MENU_DIODE: 
                                                 state = HP3478_DIOD;
                                                 printf_P(PSTR("menu: diode\r\n"));
                                                 if(!hp3478_diode_init()) HP3478_REINIT;
                                                 return 0xffff;
                         case HP3478_MENU_TEMP: 
                                                 state = HP3478_TEMP;
                                                 printf_P(PSTR("menu: temp\r\n"));
                                                 if(!hp3478_temp_init()) HP3478_REINIT;
                                                 return 0xffff;
                         case HP3478_MENU_DONE: 
                                                 state = HP3478_IDLE;
                                                 printf_P(PSTR("menu: idle\r\n"));
                                                 return 0xffff;
                         case HP3478_MENU_NOP: 
                                                 return TIMEOUT_CONT;
                         case HP3478_MENU_WAIT: 
                                                 return 100;
                 }


         case HP3478_RELS:
                 if((ev & EV_TIMEOUT) != 0) {
                  if(!hp3478_autohold_init()) HP3478_REINIT;
                  state = HP3478_AHLD;
                  return 0xffff;
                 }
                 if((sb & HP3478_SB_DREADY) == 0) return TIMEOUT_CONT;
                 if(!hp3478_get_reading(&reading, HP3478_CMD_LISTEN)) HP3478_REINIT;
                 if(reading.exp == 9) {
                  if(!hp3478_autohold_init()) HP3478_REINIT;
                  state = HP3478_AHLD;
                  return 0xffff;
                 }
                 if(!hp3478_get_status(st)) HP3478_REINIT;
                 if(!hp3478_rel_start(st[0], &reading)) HP3478_REINIT;
                 state = HP3478_RELA;
                 return 0xffff;

         case HP3478_AHLD:
         case HP3478_AHLL:
                 switch(hp3478_autohold_process(state == HP3478_AHLL, sb)) {
                         case AHLD_ERROR:        beep(0);
                                                 HP3478_REINIT;
                         case AHLD_LOCK: 
                                                 beep(1);
                                                 state = HP3478_AHLL;
                                                 return 300;
                         case AHLD_UNLOCK: 
                                                 state = HP3478_AHLD;
                                                 beep(0);
                                                 return TIMEOUT_INF;
                         default:
                                                 if(state == HP3478_AHLL) {
                                                  if(ev & EV_TIMEOUT) {
                                                   beep(0);
                                                   return TIMEOUT_INF;
                                                  }
                                                  return TIMEOUT_CONT;
                                                 }
                                                 return TIMEOUT_INF;
                 }
         case HP3478_RELA:
                  // TODO: also detect Local button?
                 if(sb & HP3478_SB_DREADY) {
                  if(!hp3478_get_reading(&reading, HP3478_CMD_LISTEN)) HP3478_REINIT;
                  if(!hp3478_rel_handle_data(&reading)) { //TODO: pass status bytes, so it knows that nothing's changed
                   if(!hp3478_cmd_P(PSTR("M20D1"), 0)) HP3478_REINIT;
                   state = HP3478_IDLE;
                  }
                  return 0xffff;
                 } 
                 return 0xffff; /* what was it? */
         case HP3478_TEMP:
                if(sb & HP3478_SB_DREADY) {
                  if(!hp3478_get_reading(&reading, HP3478_CMD_LISTEN)) HP3478_REINIT;
                  if(!hp3478_cmd_P(PSTR("K"), HP3478_CMD_CONT)) HP3478_REINIT;
                  if(!hp3478_temp_handle_data(&reading)) HP3478_REINIT;
                  return 0xffff;
                }
                return 0xffff;
         case HP3478_XOHM:
                if(sb & HP3478_SB_DREADY) {
                  if(!hp3478_get_reading(&reading, HP3478_CMD_LISTEN)) HP3478_REINIT;
                  if(!hp3478_cmd_P(PSTR("K"), HP3478_CMD_CONT)) HP3478_REINIT;
                  if(!hp3478_xohm_handle_data(&reading)) HP3478_REINIT;
                  return 0xffff;
                }
                return 0xffff;
         case HP3478_CONT:
                if(sb & HP3478_SB_DREADY) {
                  if(!hp3478_get_reading(&reading, HP3478_CMD_LISTEN)) HP3478_REINIT;
                  if(reading.value < 100000) {
                   if(!buzzer) {
                    if(!hp3478_cmd_P(PSTR("D1"), 0)) HP3478_REINIT;
                    beep(1);
                   }
                  } else if(buzzer) {
                   if(!hp3478_display_P(PSTR(" >100 OHM"), HP3478_DISP_HIDE_ANNUNCIATORS)) HP3478_REINIT;
                   beep(0);
                  }
                }
                return TIMEOUT_INF;
         case HP3478_DIOD:
                if(sb & HP3478_SB_DREADY) {
                  if(!hp3478_get_reading(&reading, HP3478_CMD_LISTEN)) HP3478_REINIT;
                  if(!hp3478_diode_handle_data(&reading)) HP3478_REINIT;
                }
                return TIMEOUT_INF;
         case HP3478_MMAX:
                { uint8_t minmax_ev;
                  uint8_t k;
                  k = hp3478_minmax_detect_key();

                 if(!hp3478_get_srq_status(&sb)) HP3478_REINIT;
                 if(k && (sb&HP3478_SB_FRPSRQ) == 0) {
                   if(!hp3478_cmd_P(PSTR("KM20D1"), 0)) HP3478_REINIT;
                   state = HP3478_IDLE;
                   return 0xffff;
                 }
                 minmax_ev = 0;
                 if(sb & HP3478_SB_DREADY) {
                   if(!hp3478_get_reading(&reading, HP3478_CMD_CONT)) HP3478_REINIT;
                   minmax_ev = hp3478_minmax_handle_data(&reading);
                 }
                 if(!hp3478_minmax_display_data(minmax_ev, sb&HP3478_SB_FRPSRQ)) HP3478_REINIT;
                 if(!hp3478_cmd_P(PSTR("M21"), HP3478_CMD_CONT)) HP3478_REINIT; /* restore mask after minmax_detect_key */
                }

                return 0xffff;
 }
 return 0xffff;
}

static void
set_defaults(uint8_t set)
{
 gpib_end_seq_tx = GPIB_END_EOI;
 gpib_end_seq_rx = GPIB_END_EOI;
 gpib_my_addr = GPIB_MY_DEFAULT_ADDRESS;
 gpib_hp3478_addr = GPIB_HP3478_DEFAULT_ADDRESS;
 hp3478_ext_enable = 0;
 uart_echo = set == 0;
}

void main(void) __attribute__((noreturn));
void main(void) 
{
  uint8_t buf[CMD_BUF_SIZE];
  uint8_t command;
  uint8_t bufPos;
  uint16_t timeout_ts = 0, timeout = 0;
  uint8_t ev;
  uint8_t ext_state;
  uint8_t b;

  led_set(LED_OFF);
  DDR(LED_PORT) = LED;
  
  TCCR0A = _BV(WGM01)|_BV(WGM00);
  OCR0A = 249; /* TOV will occur every 1ms for 16Mhz clock */
  TCCR0B = _BV(WGM02)|_BV(CS01)|_BV(CS00); /* /64 */
  TIMSK0 = _BV(TOIE0);

  TCCR1A = _BV(COM1B1)|_BV(WGM10); /* beeper */
  TCCR1B = _BV(WGM13);
  OCR1A = 10000;
  OCR1B = 5000;

  PCMSK1 = _BV(PCINT11); /* SRQ */
  PCICR = _BV(PCIE1);

  sei();
  
  command = 13; /* force line edit restart */
  set_defaults(0);
  if((b = eeprom_read_byte(&gpib_end_seq_rx_eep)) != 0xff) gpib_end_seq_rx = b;
  if((b = eeprom_read_byte(&gpib_end_seq_tx_eep)) != 0xff) gpib_end_seq_tx = b;
  if((b = eeprom_read_byte(&uart_baud_eep)) != 0xff) uart_baud = b;
  if((b = eeprom_read_byte(&gpib_my_addr_eep)) != 0xff) gpib_my_addr = b;
  if((b = eeprom_read_byte(&gpib_hp3478_addr_eep)) != 0xff) gpib_hp3478_addr = b;
  if((b = eeprom_read_byte(&uart_echo_eep)) != 0xff) uart_echo = b;
  if((b = eeprom_read_byte(&hp3478_ext_en_eep)) != 0xff) hp3478_ext_enable = b;
  if(gpib_hp3478_addr == 31) command = 'P';

  uart_init(uart_baud);
  fdevopen(uart_putchar, NULL);
  
  PORT(SRQ_PORT) |= SRQ;
  gpib_talk();

  ext_state = !hp3478_ext_enable;
  while (1) {
   // FIXME: ignore some commands so not to interrupt "EXT" mode
   if(command) command_handler(command, buf, bufPos);
   if(command) line_edit(0, buf, &bufPos); /* prepare for next command */

   ev = 0;
   if(ext_state != hp3478_ext_enable) {
    ev |= hp3478_ext_enable?EV_EXT_ENABLE:EV_EXT_DISABLE;
    ext_state = hp3478_ext_enable;
   }
   do {
    if(!uart_rx_empty()) ev |= EV_UART;
    if(gpib_srq_interrupt) {
     gpib_srq_interrupt = 0;
     if(srq()) ev |= EV_SRQ;
    }
    if(timeout != TIMEOUT_INF && (int16_t)(timeout_ts - msec_get()) <= 0) ev |= EV_TIMEOUT;
   } while(!ev);

   if(ev & (EV_SRQ|EV_TIMEOUT|EV_EXT_DISABLE|EV_EXT_ENABLE)) {
    timeout = hp3478a_handler(ev);
    if(timeout != TIMEOUT_CONT) timeout_ts = msec_get() + timeout;
   }

   if(ev & EV_UART) command = line_edit(uart_rx(), buf, &bufPos);
   else command = 0;
  }
}
