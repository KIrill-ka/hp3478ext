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
#include <ctype.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#define F_CPU 16000000UL  
#include <util/delay.h>

#include "uart.h"


/*
 TODO list

 - hp3478_get_srq_status seems to clear the GPIB status byte, but
   after some delay. Need to investigate this. For now status is explicitly
   cleared with hp3478_clear_srq.
 - hp3478_rel_start fails to update SRQ mask if status byte is not
   cleared (hp3478_clear_srq). Maybe the problem is not the status byte.
 - Display 0 with O?
 - Wait for the reading before starting rel (if you press s.trig and SRQ quickly,
   rel mode doesn't start.)
 - Display the right number of digits in hp3478_rel_handle_data.
 - Allow to save some settings in eeprom (addresses, echo, etc.).
 */

/* 
 Known caveats

 - hp3478 commands can be interrupted with ATN. For some reason, if the byte
   have been accepted by GPIB interface, it doesn't mean that the command will
   be processed. As a workaround, dummy <LF> bytes are sent so gpib_transmit waits 
   for RFD after the command.
*/

#define CMD_HISTORY_SIZE 8
#define CMD_BUF_SIZE 64

#define GPIB_TALK_ADDR_OFFSET 64
#define GPIB_LISTEN_ADDR_OFFSET 32

/* GPIB status byte / SRQ mask */
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

#define SET_PORT_PIN(PORT, PIN, V) if(V) PORT |= PIN; else PORT &= ~PIN
#define CAT(a, b) a ## b
#define PORT(X) CAT(PORT, X)
#define DDR(X) CAT(DDR, X)
#define PIN(X) CAT(PIN, X)


/* configuration constants & defaults */
#define GPIB_MY_DEFAULT_ADDRESS 21
#define GPIB_HP3478_DEFAULT_ADDRESS 23
#define GPIB_BUF_SIZE 128
#define GPIB_MAX_RECEIVE_TIMEOUT_mS 200
#define GPIB_MAX_TRANSMIT_TIMEOUT_mS 100

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
  "HP3478A GPIB-UART converter\r\n"
  "Transmit commands, OK/TIMEOUT/ERROR\r\n"
  "  D  ASCII data\r\n"
  "  M  ASCII data without EOI\r\n"
  "  C  ASCII command\r\n"
  "  TC Hex command\r\n"
  "  TD Hex data*\r\n"
  "Receive commands (receive until EOI, max 127 bytes)\r\n"
  "  X ASCII, <payload> or TIMEOUT\r\n"
  "  Y Binary**, <length><payload>\r\n"
  "  Z Hex**, <length><payload>\r\n"
  "  P Continous read (plotter mode), ESC to exit\r\n"
  "General commands\r\n"
  "  A Set/get converter GPIB address\r\n"
  "  S Get REN/SRQ/LISTEN state (1 if true)\r\n"
  "  R Set REMOTE mode (REN true)\r\n"
  "  L Set LOCAL mode (REN false)\r\n"
  "  I Generate IFC pulse\r\n"
  "  E Get/set echo (E0 off, E1 on)\r\n"
  "  Q Get/set end of line (Q0, Q1 <CR>, Q2 <LF>, Q3 <CR>+<LF>)\r\n"
  "  K Get/set HP3478A extension mode (K0 off, K1 on)\r\n"
  "  H Command history\r\n\r\n"
  "*  Add ; at the end to disable EOI\r\n"
  "** Can specify length in hex after the command (up to 80)\r\n"
;

volatile uint16_t msec_count;

static uint8_t gpib_state_listen = 0;
static uint8_t gpib_end_seq = 0;
static uint8_t gpib_my_addr = GPIB_MY_DEFAULT_ADDRESS;
static uint8_t gpib_hp3478_addr = GPIB_HP3478_DEFAULT_ADDRESS;
volatile uint8_t gpib_srq_interrupt;

#define GPIB_END_CR  1
#define GPIB_END_LF  2
#define GPIB_END_EOI 4

static char cmd_hist[CMD_BUF_SIZE*CMD_HISTORY_SIZE];
static char cmd_hist_len = 0;

enum led_mode {LED_OFF, LED_SLOW, LED_FAST};
static enum led_mode led_state;

static uint8_t hp3478_ext_enable = 1;

static uint8_t uart_echo = 1;

#define EV_TIMEOUT     1
#define EV_SRQ         2
#define EV_UART        4
#define EV_EXT_DISABLE 8
#define EV_EXT_ENABLE 16

static void 
led_set(enum led_mode m) 
{
  if(m == LED_OFF) PORT(LED_PORT) &= ~LED;
  led_state = m;
}
static inline void led_toggle(void) {PIN(LED_PORT) |= LED;}

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
    if (eoi() && (stop & GPIB_END_EOI) != 0) do_stop = 1;
    
    c = data_get();
    ndac_set(0); /* data accepted */

    buf[index++] = c;
    if (c == 10 && (stop & GPIB_END_LF) != 0) do_stop = 1;
    if (c == 13 && (stop & GPIB_END_CR) != 0) do_stop = 1;

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
  return 1;
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

ISR(PCINT1_vect) {
 gpib_srq_interrupt = 1; // FIXME: do we need an interrupt? Checking srq in main() might work better.
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

  if (len < 3) return 0;
    
  if (toupper(*buf) != 'C' && toupper(*buf) != 'D') return 0;

  if ('D' == toupper(*buf) && ';' == buf[len-1]) {
    len--;
    *send_eoi = 0;
  }
  buf++;
  len--;

  if ((len & 0x01) != 0) return 0;
    
  for (i=0; i<len; i++) if (!ishexdigit(buf[i])) return 0;

  for (i=0; i<len; i=i+2) {
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
                 if(uart_echo) printf("<GPIB> ");
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
get_read_length(const uint8_t *buf) 
{
 uint8_t l = GPIB_BUF_SIZE;
 if(ishexdigit(buf[1])) l = hex2dec(buf[1]);
 if(ishexdigit(buf[2])) {
  l <<= 4;
  l |= hex2dec(buf[2]);
 }
 if(l > GPIB_BUF_SIZE) l = GPIB_BUF_SIZE;
 return l;
}


static void 
get_set_opt(const uint8_t *buf, uint8_t len, uint8_t max, uint8_t *opt)
{
 uint16_t v;
 uint8_t i;

 if(len == 1) {
  printf("%d\r\n", *opt);
  return;
 }

 for(i = 1, v = 0; i < len; i++) {
  uint8_t c = buf[i];
  v += v*10 + (c-'0');
  if(c > '9' || c < '0' || v > max) {
   printf_P(PSTR("ERROR\r\n"));
   return;
  }
 }
  
 *opt = (uint8_t)v;
 printf_P(PSTR("OK\r\n"));
}

static void 
command_handler(uint8_t command, uint8_t *buf, uint8_t len)
{
  uint8_t gpibBuf[GPIB_BUF_SIZE];
  uint8_t gpibIndex;
  uint8_t i;
  uint8_t send_eoi;
  uint8_t result;

   switch(command) {
           case 'D': /* send with EOI */
           case 'M': /* send without EOI */
                   if (gpib_state_listen) {
                    printf_P(PSTR("ERROR\r\n"));	  
                    break;
                   }
                   result = gpib_transmit(buf+1, len-1, gpib_end_seq | (command == 'D' ? GPIB_END_EOI : 0) ); 
                   if (result) printf_P(PSTR("OK\r\n"));
                   else printf_P(PSTR("TIMEOUT\r\n"));

                   break;
           case 'C': /* send command */
                   for (i=1; i<len; i++) {
                    if (buf[i] == '?' || buf[i] == 64+gpib_my_addr) {
                     gpib_state_listen = 0;
                     led_set(LED_OFF);
                    } else if (buf[i] == 32+gpib_my_addr) {
                     gpib_state_listen = 1;
                     led_set(LED_FAST);
                    }
                   }

                   gpib_talk();

                   set_atn(1);
                   result = gpib_transmit(buf+1, len-1, gpib_end_seq | GPIB_END_EOI);

                   if (result) printf_P(PSTR("OK\r\n"));
                   else printf_P(PSTR("TIMEOUT\r\n"));

                   set_atn(0);

                   if (gpib_state_listen) gpib_listen();
                   else gpib_talk();

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
                   if (gpib_state_listen) {
                    gpib_state_listen = 0;
                    led_set(LED_OFF);
                    gpib_talk();
                   }
                   printf_P(PSTR("OK\r\n"));
                   break;

           case 'S':
                   uart_tx(ren()?'1':'0');
                   uart_tx(srq()?'1':'0');
                   uart_tx(gpib_state_listen?'1':'0');
                   uart_tx(13);
                   uart_tx(10);
                   break;

           case 'P':
                   led_set(LED_SLOW);
                   gpib_listen();

                   while (1) {
                    uint8_t c;
                    if (!uart_rx_empty()) c = uart_rx();
                    if(c == 27) break;
                    result = gpib_receive(gpibBuf, 1, &gpibIndex, 0);
                    for (i=0; i < gpibIndex; i++) uart_tx(gpibBuf[i]);
                    if(gpibIndex == 0) _delay_ms(10);
                   }
                   gpib_state_listen = 0;
                   gpib_talk();
                   led_set(LED_OFF);
                   break;
           case 'X': /* ascii receive */
                   if (!gpib_state_listen) gpib_listen();
                   gpib_receive(gpibBuf, GPIB_BUF_SIZE-1, &gpibIndex, GPIB_END_EOI);

                   if (gpibIndex != 0) {
                    gpibBuf[gpibIndex] = 0;
                    printf("%s", gpibBuf);
                   } else printf_P(PSTR("TIMEOUT\r\n"));

                   if (!gpib_state_listen) gpib_talk();
                   break;
           case 'Y': /* binary receive */
                   if (!gpib_state_listen) gpib_listen();
                   result = gpib_receive(gpibBuf, get_read_length(buf), &gpibIndex, GPIB_END_EOI);
                   uart_tx(gpibIndex);
                   for (i=0; i<gpibIndex; i++) uart_tx(gpibBuf[i]);

                   if (!gpib_state_listen) gpib_talk();
                   break;
           case 'Z': /* hex receive */
                   if (!gpib_state_listen) gpib_listen();
                   result = gpib_receive(gpibBuf, get_read_length(buf), &gpibIndex, GPIB_END_EOI);

                   printf("%02X", gpibIndex);
                   for (i=0; i<gpibIndex; i++)
                    printf("%02X", gpibBuf[i]);
                   printf("\r\n");

                   if (!gpib_state_listen) gpib_talk();
                   break;
           case '?':
                   printf_P(help);
                   break;
           case 'E':
                   get_set_opt(buf, len, 1, &uart_echo);
                   break;

           case 'H':
                   for (i=0; i < cmd_hist_len; i++)
                    printf("%d: %s\r\n", i, cmd_hist+i*CMD_BUF_SIZE);
                   break;
           case 'A':
                   get_set_opt(buf, len, 30, &gpib_my_addr);
                   break;
           case 'Q':
                   get_set_opt(buf, len, 3, &gpib_end_seq);
                   break;

           case 'T':
                   if (!convert_hex_message(&buf[1], len-1, gpibBuf, &gpibIndex, &send_eoi)) {
                    printf_P(PSTR("ERROR\r\n"));
                    break;
                   }
                   if ('D' == toupper(buf[1])) {
                    /* send data */
                    result = gpib_transmit(gpibBuf, gpibIndex, send_eoi); 
                    if (result) printf_P(PSTR("OK\r\n"));
                    else printf_P(PSTR("TIMEOUT\r\n"));
                   } else {
                    /* send command  */
                    for (i=0; i<gpibIndex; i++) {
                     if (gpibBuf[i] == '?' || gpibBuf[i] == gpib_my_addr+64) {
                      gpib_state_listen = 0;
                      led_set(LED_OFF);
                     } else if (gpibBuf[i] == gpib_my_addr+32) {
                      gpib_state_listen = 1;
                      led_set(LED_FAST);
                     }
                    }

                    gpib_talk();

                    set_atn(1);
                    _delay_us(100);
                    result = gpib_transmit(gpibBuf, gpibIndex, 0);

                    if (result) printf_P(PSTR("OK\r\n"));
                    else printf_P(PSTR("TIMEOUT\r\n"));

                    set_atn(0);

                    if (gpib_state_listen) gpib_listen();

                   }      
                   break;
           case 'K':
                   get_set_opt(buf, len, 1, &hp3478_ext_enable);
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

static uint8_t 
hp3478_clear_srq(void)
{
 uint8_t cmd[2];

 set_ren(1);
 cmd[0] = gpib_hp3478_addr+GPIB_LISTEN_ADDR_OFFSET;
 cmd[1] = gpib_my_addr+GPIB_TALK_ADDR_OFFSET;
 set_atn(1);
 if(!gpib_transmit(cmd, 2, 0)) goto fail;
 set_atn(0);
 cmd[0] = 'K'; /* clear SRQ bits */
 if(!gpib_transmit(cmd, 1, GPIB_END_LF)) goto fail;
 set_ren(0);
 return 1;
fail:
 set_ren(0);
 set_atn(0);
 return 0;
}

static uint8_t 
hp3478_get_reading(struct hp3478_reading *reading)
{
 uint8_t cmd[2];
 uint8_t buf[13];
 uint8_t sign;
 int32_t v;
 uint8_t i;
 uint8_t len;

 cmd[0] = gpib_my_addr+GPIB_LISTEN_ADDR_OFFSET;
 cmd[1] = gpib_hp3478_addr+GPIB_TALK_ADDR_OFFSET;
 set_atn(1);
 if(!gpib_transmit(cmd, 2, 0)) goto fail;
 set_atn(0);

 gpib_listen();
 if(!gpib_receive(buf, sizeof(buf), &len, GPIB_END_EOI)) goto fail;
 gpib_talk();

 set_atn(1);
 cmd[0] = '_';
 if(!gpib_transmit(cmd, 1, 0)) goto fail;
 set_atn(0);

 i = 0;
 sign = buf[0] == '-';
 v = 0;
 for(++i; i < len; i++) {
  if(buf[i] == 'E') break;
  if(buf[i] == '.') reading->dot = i-1;
  else v = v*10 + (buf[i]-'0');
 }
 i++;
 if(len-i < 2) goto fail;
 reading->value = sign ? -v : v;
 sign = buf[i++] == '-';
 reading->exp = sign ? '0'-buf[i] : buf[i]-'0';
 return 1;

fail:
 set_atn(0);
 gpib_talk();
 return 0;
}


static uint8_t 
hp3478_init_srq(void)
{
 uint8_t cmd[4];

 printf("init srq\r\n");
 gpib_state_listen = 0;
 gpib_talk();

 set_ren(1);
 cmd[0] = gpib_my_addr+GPIB_TALK_ADDR_OFFSET;
 cmd[1] = gpib_hp3478_addr+GPIB_LISTEN_ADDR_OFFSET;
 set_atn(1);
 if(!gpib_transmit(cmd, 2, 0)) {
  printf("init srq: listen fail\r\n");
  goto fail;
 }
 set_atn(0);
 cmd[0] = 'M'; /* set SRQ mask */
 cmd[1] = '2'; /* enable SRQ button */
 cmd[2] = '0';
 cmd[3] = 'K'; /* clear SRQ bits */
 if(!gpib_transmit(cmd, 4, GPIB_END_LF)) {
  printf("init srq: mask fail\r\n");
  goto fail;
 }
 set_ren(0);

 set_atn(1);
 cmd[0] = '?';
 if(!gpib_transmit(cmd, 1, 0)) goto fail;
 set_atn(0);

 return 1;
fail:
 set_ren(0);
 set_atn(0);
 return 0;
}

static uint8_t
hp3478_disable_srq(void)
{
 uint8_t cmd[3];
 set_ren(1);
 cmd[0] = gpib_my_addr+GPIB_TALK_ADDR_OFFSET;
 cmd[1] = gpib_hp3478_addr+GPIB_LISTEN_ADDR_OFFSET;
 set_atn(1);
 if(!gpib_transmit(cmd, 2, 0)) goto fail;
 set_atn(0);
 cmd[0] = 'M'; /* set SRQ mask */
 cmd[1] = '0';
 cmd[2] = '0';
 if(!gpib_transmit(cmd, 3, GPIB_END_LF)) goto fail;
 set_ren(0);

 set_atn(1);
 cmd[0] = '?';
 if(!gpib_transmit(cmd, 1, 0)) goto fail;
 set_atn(0);

 return 1;
fail:
 set_ren(0);
 set_atn(0);
 return 0;
}

static uint8_t
hp3478_get_srq_status(uint8_t *sb)
{
 uint8_t cmd[3];
 uint8_t rl;

 cmd[0] = 24; /* serial poll enable */
 cmd[1] = gpib_hp3478_addr+GPIB_TALK_ADDR_OFFSET;
 cmd[2] = gpib_my_addr+GPIB_LISTEN_ADDR_OFFSET;
 set_atn(1);
 if(!gpib_transmit(cmd, 3, 0)) goto fail;
 set_atn(0);
 gpib_listen();
 if(!gpib_receive(sb, 1, &rl, 0)) goto fail;
 gpib_talk();
 set_atn(1);
 cmd[0] = 25; /* serial poll disable */
 cmd[1] = '_';
 if(!gpib_transmit(cmd, 2, 0)) goto fail;
 set_atn(0);
 return 1;
fail:
 gpib_talk();
 set_atn(0);
 return 0;
}

static uint8_t
hp3478_get_status(uint8_t st[5])
{
 uint8_t cmd[3];
 uint8_t rl;

 set_ren(1);
 cmd[0] = gpib_hp3478_addr+GPIB_LISTEN_ADDR_OFFSET;
 cmd[1] = gpib_my_addr+GPIB_TALK_ADDR_OFFSET;
 set_atn(1);
 if(!gpib_transmit(cmd, 2, 0)) goto fail;
 set_atn(0);
 cmd[0] = 'B';
 if(!gpib_transmit(cmd, 1, GPIB_END_LF)) goto fail;
 set_ren(0);
 cmd[0] = gpib_my_addr+GPIB_LISTEN_ADDR_OFFSET;
 cmd[1] = gpib_hp3478_addr+GPIB_TALK_ADDR_OFFSET;
 set_atn(1);
 if(!gpib_transmit(cmd, 2, 0)) goto fail;
 set_atn(0);
 gpib_listen();
 rl = 0;
 if(!gpib_receive(st, 5, &rl, GPIB_END_EOI)) {
  printf("received: %d\r\n", rl);
  goto fail;
 }
 gpib_talk();
 set_atn(1);
 cmd[0] = '_';
 if(!gpib_transmit(cmd, 1, 0)) goto fail;
 set_atn(0);
 if(rl != 5) return 0;
 return 1;
fail:
 gpib_talk();
 set_atn(0);
 set_ren(0);
 return 0;
}
static uint8_t hp3478_rel_mode;
static struct hp3478_reading hp3478_rel_ref;
static uint8_t
hp3478_rel_start(uint8_t st1, const struct hp3478_reading *r)
{
 uint8_t cmd[5];
 set_ren(1);
 cmd[0] = gpib_hp3478_addr+GPIB_LISTEN_ADDR_OFFSET;
 cmd[1] = gpib_my_addr+GPIB_TALK_ADDR_OFFSET;
 set_atn(1);
 if(!gpib_transmit(cmd, 2, 0)) goto fail;
 set_atn(0);
 cmd[0] = 'M'; /* set SRQ mask */
 cmd[1] = '2'; /* FP SRQ */
 cmd[2] = '1'; /* data ready */
 cmd[3] = 'T';
 cmd[4] = '1'; /* internal trigger */
 if(!gpib_transmit(cmd, 5, GPIB_END_LF)) goto fail;
 set_ren(0);
 hp3478_rel_mode = st1;
 hp3478_rel_ref = *r;
 return 1;
fail:
 set_ren(0);
 set_atn(0);
 return 0;
}

static uint8_t
hp3478_rel_stop(void)
{
 uint8_t cmd[5];
 set_ren(1);
 cmd[0] = gpib_my_addr+GPIB_TALK_ADDR_OFFSET;
 cmd[1] = gpib_hp3478_addr+GPIB_LISTEN_ADDR_OFFSET;
 set_atn(1);
 if(!gpib_transmit(cmd, 2, 0)) goto fail;
 set_atn(0);
 cmd[0] = 'M'; /* set SRQ mask */
 cmd[1] = '2'; /* FP SRQ */
 cmd[2] = '0'; /* data ready */
 cmd[3] = 'D';
 cmd[4] = '1';
 if(!gpib_transmit(cmd, 5, GPIB_END_LF)) goto fail;
 set_ren(0);
 set_atn(1);
 cmd[0] = '?';
 if(!gpib_transmit(cmd, 1, 0)) goto fail;
 set_atn(0);
 return 1;
fail:
 set_ren(0);
 set_atn(0);
 return 0;
}


static uint8_t
hp3478_display(const char display[], uint8_t len)
{
 uint8_t cmd[2];
 set_ren(1);
 cmd[0] = gpib_hp3478_addr+GPIB_LISTEN_ADDR_OFFSET;
 cmd[1] = gpib_my_addr+GPIB_TALK_ADDR_OFFSET;
 set_atn(1);
 if(!gpib_transmit(cmd, 2, 0)) goto fail;
 set_atn(0);
 cmd[0] = 'D'; /* display */
 cmd[1] = '3'; /* hide annunciators */
 if(!gpib_transmit(cmd, 2, 0)) goto fail;
 if(!gpib_transmit((const uint8_t*)display, len, GPIB_END_LF|GPIB_END_CR)) goto fail;
 set_ren(0);

 return 1;
fail:
 set_ren(0);
 set_atn(0);
 return 0;
}

static uint8_t
hp3478_rel_handle_data(struct hp3478_reading *r)
{
 struct hp3478_reading ref, in, out;
 int8_t e_ref, e_in;
 //char display[13];
 char display[14];
 char exp_char;
 const char *m;
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

 if(out.value >= 0) display[0] = '+';
 else {
  display[0] = '-';
  out.value = -out.value;
 }
  
 for(i = 7; i > 0; i--) {
  display[i] = out.value % 10 + '0';
  out.value /= 10;
  if(i == in.dot+2) display[--i] = '.';
 }
 switch(out.exp) {
   case -3: exp_char = 'M'; break;
   case 0: exp_char = ' '; break;
   case 3: exp_char = 'K'; break;
   case 6: exp_char = 'M'; break;
   default: exp_char = '?';
 }
 display[8] = exp_char;
 switch(hp3478_rel_mode & HP3478_ST_FUNC) {
         case HP3478_ST_FUNC_DCV: m = PSTR("VDC"); break;
         case HP3478_ST_FUNC_ACV: m = PSTR("VAC"); break;
         case HP3478_ST_FUNC_2WOHM:
         case HP3478_ST_FUNC_4WOHM: m = PSTR("OHM"); break;
         case HP3478_ST_FUNC_DCA: m = PSTR("ADC"); break;
         case HP3478_ST_FUNC_ACA: m = PSTR("ACA"); break;
         default: m = PSTR("???"); 
 }
 memcpy_P(display+9, m, 3);

 display[12] = '*';
 display[13] = '\0'; // FIXME: remove, this is for printf
 printf_P(PSTR("rel reading %ld>>>%ld, %d, %s\r\n"), hp3478_rel_ref.value, r->value, out.exp, display);
 return hp3478_display(display, sizeof(display)-1);
}


#define HP3478_REINIT do { \
 state = HP3478_INIT; \
 return 250; \
} while(0)

static uint16_t
hp3478a_handler(uint8_t ev)
{
#define HP3478_INIT    1
#define HP3478_IDLE    2
#define HP3478_RELA    3
#define HP3478_DISABLE 4
 static uint8_t state = HP3478_INIT;
 uint8_t sb;
 uint8_t st[5];
 struct hp3478_reading reading;
 switch(state) {
         case HP3478_DISABLE:
                 if((ev & EV_EXT_ENABLE) == 0) return 0xffff;
                 state = HP3478_INIT;
         case HP3478_INIT:
                 if(ev & EV_EXT_DISABLE) {
                  state = HP3478_DISABLE;
                  return 0xffff;
                 }
                 if(hp3478_init_srq()) {
                  state = HP3478_IDLE;
                  return 0xffff;
                 }
                 return 2000; /* retry initialization after 2 sec */

         case HP3478_IDLE:
                 if(ev & EV_EXT_DISABLE) {
                   hp3478_disable_srq();
                   state = HP3478_DISABLE;
                   return 0xffff;
                 }
                 if(!srq()) return 0xffff;
                 if(!hp3478_get_srq_status(&sb)) HP3478_REINIT;
                 if((sb & HP3478_SB_FRPSRQ) == 0) {
                  if(!hp3478_clear_srq()) HP3478_REINIT;
                  return 0xffff;
                 }
                 if((sb & HP3478_SB_DREADY) != 0)
                  if(!hp3478_get_reading(&reading)) HP3478_REINIT;
                 if(!hp3478_clear_srq()) HP3478_REINIT;
                 if(!hp3478_get_status(st)) HP3478_REINIT;
                 printf("got status %x %x\r\n", (unsigned)st[1], (unsigned)sb);
                 if((st[1] & HP3478_ST_INT_TRIGGER) == 0 && (sb & HP3478_SB_DREADY) != 0) {
                  printf("starting rel\r\n");
                  if(!hp3478_rel_start(st[0], &reading)) HP3478_REINIT;
                  state = HP3478_RELA;
                  return 0xffff;
                 }
                 return 0xffff;

         case HP3478_RELA:
                 if(ev & EV_EXT_DISABLE) {
                   hp3478_rel_stop();
                   hp3478_disable_srq();
                   state = HP3478_DISABLE;
                   return 0xffff;
                 }
                 if(!srq()) return 0xffff;
                 if(!hp3478_get_srq_status(&sb)) HP3478_REINIT;
                 if(sb & HP3478_SB_FRPSRQ) {
                  // TODO: also detect Local button?
                  if(!hp3478_clear_srq()) HP3478_REINIT;
                  if(!hp3478_rel_stop()) HP3478_REINIT;
                  state = HP3478_IDLE;
                  return 0xffff;
                 }
                 if(sb & HP3478_SB_DREADY) {
                  if(!hp3478_get_reading(&reading)) HP3478_REINIT;
                  if(!hp3478_clear_srq()) HP3478_REINIT;
                  if(!hp3478_rel_handle_data(&reading)) { //TODO: pass status bytes, so it knows that nothing's changed
                   if(!hp3478_rel_stop()) HP3478_REINIT;
                   state = HP3478_IDLE;
                  }
                  return 0xffff;
                 }
                 return 0xffff; /* what was it? */
 }
 return 0xffff;
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

  led_set(LED_OFF);
  DDR(LED_PORT) = LED;
  
  TCCR0A = _BV(WGM01)|_BV(WGM00);
  OCR0A = 249; /* TOV will occur every 1ms for 16Mhz clock */
  TCCR0B = _BV(WGM02)|_BV(CS01)|_BV(CS00); /* /64 */
  TIMSK0 = _BV(TOIE0);

  PCMSK1 = _BV(PCINT11);
  PCICR = _BV(PCIE1);

  sei();
  
  uart_init();
  fdevopen(uart_putchar, NULL);
  
  command = 13; /* force line edit restart */

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
     ev |= EV_SRQ;
    }
    if(timeout != 0xffff && timeout_ts - msec_get() <= 0) ev |= EV_TIMEOUT;
   } while(!ev);

   if(ev & (EV_SRQ|EV_TIMEOUT|EV_EXT_DISABLE|EV_EXT_ENABLE)) {
    timeout = hp3478a_handler(ev);
    timeout_ts = msec_get() + timeout;
   }

   if(ev & EV_UART) command = line_edit(uart_rx(), buf, &bufPos);
   else command = 0;
  }
}
