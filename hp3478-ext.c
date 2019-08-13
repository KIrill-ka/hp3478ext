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
#include "eepmap.h"


/*
 TODO list

 - Display 0 with O?
 - Implement unbuffered binary write TUD using escape-sequence as stop
 - Implement unbuffered binary read TUD using escape-sequence as stop
 - Save & restore "ext" functions using presets.
 - Add dBm measurements in ACV.
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
 - Reading results of "B" and "S" commands (and possibly all reads) clear DREADY status bit.
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

#define HP3478_ST_FUNC       (7<<5)
#define HP3478_ST_FUNC_DCV   (1<<5)
#define HP3478_ST_FUNC_ACV   (2<<5)
#define HP3478_ST_FUNC_2WOHM (3<<5)
#define HP3478_ST_FUNC_4WOHM (4<<5)
#define HP3478_ST_FUNC_DCA   (5<<5)
#define HP3478_ST_FUNC_ACA   (6<<5)
#define HP3478_ST_FUNC_XOHM  (7<<5)

/* status byte 2 */
#define HP3478_ST_INT_TRIGGER  (1<<0)
#define HP3478_ST_AUTORANGE    (1<<1)
#define HP3478_ST_AUTOZERO     (1<<2)
#define HP3478_ST_50HZ         (1<<3)
#define HP3478_ST_FRONT_INP_SW (1<<4)
#define HP3478_ST_CAL_ENABLED  (1<<5)
#define HP3478_ST_EXT_TRIGGER  (1<<6)

#define SET_PORT_PIN(PORT, PIN, V) if(V) PORT |= PIN; else PORT &= ~PIN
#define CAT(a, b) a ## b
#define PORT(X) CAT(PORT, X)
#define DDR(X) CAT(DDR, X)
#define PIN(X) CAT(PIN, X)

#if 0
#define DIAG_STR(s) #s
#define DIAG_JOINSTR(x,y) DIAG_STR(x ## y)
#define DIAG_DO_PRAGMA(x) _Pragma (#x)
#define DIAG_PRAGMA(compiler,x) DIAG_DO_PRAGMA(compiler diagnostic x)

#define DISABLE_WARNING(gcc_option) DIAG_PRAGMA(GCC,push) DIAG_PRAGMA(GCC,ignored DIAG_JOINSTR(-W,gcc_option))
#define RESTORE_WARNING() DIAG_PRAGMA(GCC,pop)
#endif

/* configuration constants & defaults */
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
 5   | EOI   | End Of Identity    | Talker     | PB3  15 | 11
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
#define BUZZ _BV(PB2)
#define BUZZ_PORT B

static void cfg_data_in(void) {
  DDRD &= ~(_BV(PD2)|_BV(PD3)|_BV(PD4)|_BV(PD5)|_BV(PD6)|_BV(PD7));
  DDRB &= ~(_BV(PB0)|_BV(PB1));
}
static void cfg_data_out(void) {
  /* DDRD |= (_BV(PD2)|_BV(PD3)|_BV(PD4)|_BV(PD5)|_BV(PD6)|_BV(PD7));
     DDRB |= (_BV(PB0)|_BV(PB1)); */
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

static inline void eoi_set(uint8_t x) {SET_PORT_PIN(DDR(EOI_PORT), EOI, (x));}
static inline void dav_set(uint8_t x) {SET_PORT_PIN(DDR(DAV_PORT), DAV, (x));}
static inline void nrfd_set(uint8_t x) {SET_PORT_PIN(DDR(NRFD_PORT), NRFD, (x));}
static inline void ndac_set(uint8_t x) {SET_PORT_PIN(DDR(NDAC_PORT), NDAC, (x));}
static inline void SetIFC(uint8_t x) {SET_PORT_PIN(DDR(IFC_PORT), IFC, !(x));}
static inline void set_atn(uint8_t x) {
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
static enum led_mode led_state = LED_OFF;

static uint8_t hp3478_ext_enable;
static uint16_t hp3478_init_mode;

static uint8_t uart_echo;
static uint8_t uart_baud;

static uint16_t buzz_period;
static uint8_t buzz_duty;
static uint16_t cont_threshold;
static uint16_t cont_buzz_t1;
static uint16_t cont_buzz_t2;
static uint16_t cont_buzz_p1;
static uint16_t cont_buzz_p2;
static uint8_t cont_buzz_d1;
static uint8_t cont_buzz_d2;
static uint8_t cont_latch;
static uint8_t cont_range;

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

static uint8_t cont_latch_dncnt;
static uint8_t buzzer = 0;
static void 
beep(uint16_t period, uint8_t duty)
{
 if(duty) {
  if(!period) PORT(BUZZ_PORT) |= BUZZ;
  else {
   OCR1A = period;
   OCR1B = (uint32_t)period*duty >> 8;
   TCCR1A = _BV(COM1B1)|_BV(WGM10);
   TCCR1B = _BV(WGM13)|_BV(CS10);
  }
 }
 buzzer = 1;
}

static void 
beep_off(void)
{
 buzzer = 0;

 TCCR1B = 0;
 TCCR1A = 0;
 PORT(BUZZ_PORT) &= ~BUZZ;
}

static void
cont_beep(uint16_t val)
{
 uint16_t period, p1, p2;
 uint16_t t1, t2;
 uint8_t duty, d1, d2;
 p1 = cont_buzz_p1;
 p2 = cont_buzz_p2;
 d1 = cont_buzz_d1;
 d2 = cont_buzz_d2;
 t1 = cont_buzz_t1;
 t2 = cont_buzz_t2;

 if(val <= t1) {
  period = p1;
  duty = d1;
 } else if(val >= t2) {
  period = p2;
  duty = d2;
 } else {
  period = (uint16_t)((uint32_t)(p2-p1)*(val-t1)/(t2-t1)) + p1;
  duty = (uint8_t)((uint16_t)(d2-d1)*(val-t1)/(t2-t1)) + d1;
 }
 beep(period, duty);
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
        cfg_data_in();
        return i;
      }
    }
    
    dav_set(1);
   
    while (ndac()) { /* waiting for high on NDAC */
      if ((uint8_t)((uint8_t)msec_count-ts) > GPIB_MAX_TRANSMIT_TIMEOUT_mS) {
        eoi_set(0);
        dav_set(0);
        cfg_data_in();
        return i;
      }
    }
    
    dav_set(0);
  }
  eoi_set(0);
  cfg_data_in();
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


struct opt_info {
 char name[16];
 uint16_t max;
 uint16_t def;
#define OPT_INFO_W16 1
 uint8_t flags;
 void *addr;
 void *addr_eep;
};

const struct opt_info PROGMEM opts[] = {
 {.name = "X",          
  .max = 1, .def = EEP_DEF0_HP3478_EXT_EN,
  .addr = &hp3478_ext_enable,    .addr_eep = (void*)EEP_ADDR_HP3478_EXT_EN},
 {.name = "I",
  .max = 1, .def = EEP_DEF0_UART_ECHO,
  .addr = &uart_echo,            .addr_eep = (void*)EEP_ADDR_UART_ECHO},
 {.name = "C",
  .max = 30, .def = EEP_DEF0_GPIB_MY_ADDR,
  .addr = &gpib_my_addr,         .addr_eep = (void*)EEP_ADDR_GPIB_MY_ADDR},
 {.name = "D",
  .max = 31, .def = EEP_DEF0_GPIB_HP3478_ADDR,
  .addr = &gpib_hp3478_addr,     .addr_eep = (void*)EEP_ADDR_GPIB_HP3478_ADDR},
 {.name = "R",
  .max = 7,  .def = EEP_DEF0_GPIB_END_SEQ_RX,
  .addr = &gpib_end_seq_rx,      .addr_eep = (void*)EEP_ADDR_GPIB_END_SEQ_RX},
 {.name = "T",
  .max = 7,  .def = EEP_DEF0_GPIB_END_SEQ_TX,
  .addr = &gpib_end_seq_tx,      .addr_eep = (void*)EEP_ADDR_GPIB_END_SEQ_TX},
 {.name = "B",
  .max = 4, .def = EEP_DEF0_UART_BAUD,
  .addr = &uart_baud,            .addr_eep = (void*)EEP_ADDR_UART_BAUD},
 {.name = "init_mode",
  .max = 0x7fff,.def = EEP_DEF0_MODE,         .flags = OPT_INFO_W16,
  .addr = &hp3478_init_mode,     .addr_eep = (void*)EEP_ADDR_MODE},
 {.name = "beep_period",
  .max = 65534,  .def = EEP_DEF0_BEEP_PERIOD, .flags = OPT_INFO_W16,
  .addr = &buzz_period,          .addr_eep = (void*)EEP_ADDR_BEEP_PERIOD},
 {.name = "beep_duty",
  .max = 127,  .def = EEP_DEF0_BEEP_DUTY,
  .addr = &buzz_duty,            .addr_eep = (void*)EEP_ADDR_BEEP_DUTY},
 {.name = "cont_thr",
  .max = 3000,  .def = EEP_DEF0_CONT_THRESHOLD, .flags = OPT_INFO_W16,
  .addr = &cont_threshold,       .addr_eep = (void*)EEP_ADDR_CONT_THRESHOLD},
 {.name = "cont_latch",
  .max = 100,  .def = EEP_DEF0_CONT_LATCH,
  .addr = &cont_latch,           .addr_eep = (void*)EEP_ADDR_CONT_LATCH},
 {.name = "cont_range",
  .max = 6,  .def = EEP_DEF0_CONT_RANGE,
  .addr = &cont_range,           .addr_eep = (void*)EEP_ADDR_CONT_RANGE},
 {.name = "cont_beep_ta",
  .max = 3000,  .def = EEP_DEF0_CONT_BEEP_T1, .flags = OPT_INFO_W16,
  .addr = &cont_buzz_t1,         .addr_eep = (void*)EEP_ADDR_CONT_BEEP_T1},
 {.name = "cont_beep_tb",
  .max = 3000,  .def = EEP_DEF0_CONT_BEEP_T2, .flags = OPT_INFO_W16,
  .addr = &cont_buzz_t2,         .addr_eep = (void*)EEP_ADDR_CONT_BEEP_T2},
 {.name = "cont_beep_pa",
  .max = 65534, .def = EEP_DEF0_CONT_BEEP_P1, .flags = OPT_INFO_W16,
  .addr = &cont_buzz_p1,         .addr_eep = (void*)EEP_ADDR_CONT_BEEP_P1},
 {.name = "cont_beep_pb",
  .max = 65534, .def = EEP_DEF0_CONT_BEEP_P2, .flags = OPT_INFO_W16,
  .addr = &cont_buzz_p2,         .addr_eep = (void*)EEP_ADDR_CONT_BEEP_P2},
 {.name = "cont_beep_da",
  .max = 127,   .def = EEP_DEF0_CONT_BEEP_D1,
  .addr = &cont_buzz_d1,         .addr_eep = (void*)EEP_ADDR_CONT_BEEP_D1},
 {.name = "cont_beep_db",
  .max = 127,   .def = EEP_DEF0_CONT_BEEP_D2,
  .addr = &cont_buzz_d2,         .addr_eep = (void*)EEP_ADDR_CONT_BEEP_D2}
};

static uint8_t 
get_opt_info(const uint8_t *buf, uint8_t len, struct opt_info *ret)
{
 uint8_t i;
 uint8_t cnt = 0;

 for(i = 0; i < len; i++) {
  uint8_t c = buf[i];
  if(!(c >= 'a' && c <= 'z') && !(c >= 'A' && c <= 'Z') && c != '_') break;
 }
 len = i;
 if(len == 0 || len >= sizeof(opts->name)) return 0;


 for(i = 0; i < sizeof(opts)/sizeof(opts[0]); i++) {
  if(!strncmp_P((const char*)buf, opts[i].name, len)) {
   memcpy_P(ret, opts+i, sizeof(*opts));
   cnt++;
  }
 }
 if(cnt != 1) return 0;

 return len;
}


static uint8_t 
get_set_opt(const uint8_t *buf, uint8_t len)
{
 uint16_t v;
 uint8_t i, w = 0;
 struct opt_info opt;

 if(len == 0) {
   printf_P(PSTR("ERROR\r\n"));
   return 0;
 }
 switch(buf[0]) {
#if 0
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
#endif
          case '0':
          case '1':
                  set_defaults(buf[0]-'0');
                  printf_P(PSTR("OK\r\n"));
                  return 1;
          case '?':
                  printf_P(opt_help);
                  return 0;
          default:
                  i = get_opt_info(buf, len, &opt);
                  if(!i) {
                   printf_P(PSTR("WRONG OPTION\r\n"));
                   return 0;
                  }
                  buf += i;
                  len -= i;
                  
 }

 if(len == 0) {
  unsigned val;
  if(opt.flags & OPT_INFO_W16) val = *(uint16_t*)opt.addr;
  else val = *(uint8_t*)opt.addr;
  printf_P(PSTR("%u\r\n"), val);
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
 if(v > opt.max) {
  printf_P(PSTR("ERROR\r\n"));
  return 0;
 }

 if(opt.flags & OPT_INFO_W16) {
  *(uint16_t*)opt.addr = (uint16_t)v;
  if(w) eeprom_write_word(opt.addr_eep, (uint16_t)v);
 } else {
  *(uint8_t*)opt.addr = (uint8_t)v;
  if(w) eeprom_write_byte(opt.addr_eep, (uint8_t)v);
 }
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
                   if(gpib_end_seq_tx & GPIB_END_CR) len++;
                   if(gpib_end_seq_tx & GPIB_END_LF) len++;
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
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
                     uart_tx(result);
#pragma GCC diagnostic pop
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

static uint8_t errcode;
static uint8_t errcode2;
static uint8_t errcode3;
static uint8_t errcode4;
#define L2_ERRCODE(X) do {errcode2 = (X);} while(0)
#define L3_ERRCODE(X) do {errcode3 = (X);} while(0)
#define L4_ERRCODE(X) do {errcode4 = (X);} while(0)

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
   if(!gpib_transmit_b(cmd1, 2, 0)) {
    errcode = 1;
    goto fail;
   }
   set_atn(0);
 }
 if(!gpib_transmit_b(cmd, len, (flags & HP3478_CMD_NO_LF) ? 0 : GPIB_END_LF)) {
  errcode = 2;
  goto fail;
 }
 if((flags & HP3478_CMD_REMOTE) == 0) set_ren(0);
 if((flags & HP3478_CMD_TALK) == 0) {
  cmd1[0] = '?';
  set_atn(1);
  if(!gpib_transmit_b(cmd1, 1, 0)) {
   errcode = 3;
   goto fail;
  }
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
   if(!gpib_transmit_b(cmd1, 2, 0)) {
    errcode = 1;
    goto fail;
   }
   set_atn(0);
 }
 if(!gpib_transmit_P((const uint8_t*)cmd, strlen_P(cmd), GPIB_END_LF)) {
  errcode = 2;
  goto fail;
 }
 if((flags & HP3478_CMD_REMOTE) == 0) set_ren(0);
 if((flags & HP3478_CMD_TALK) == 0) {
  cmd1[0] = '?';
  set_atn(1);
  if(!gpib_transmit_b(cmd1, 1, 0)) {
   errcode = 3;
   goto fail;
  }
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
 if(!gpib_transmit_b(cmd, 3, 0)) {
  errcode = 4;
  goto fail;
 }
 set_atn(0);
 gpib_listen();
 gpib_receive(sb, 1, &rl, 0);
 if(rl != 1) {
  errcode = 5;
  goto fail;
 }
 gpib_talk();
 set_atn(1);
 cmd[0] = 25; /* serial poll disable */
 cmd[1] = '_';
 if(!gpib_transmit_b(cmd, 2, 0)) {
  errcode = 6;
  goto fail;
 }
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
   errcode = 7;
   goto fail;
  }
  set_atn(0);
  gpib_listen();
 }
 if(gpib_receive(buf, buf_sz, rl, GPIB_END_EOI) != GPIB_END_EOI) {
  errcode = 8;
  goto fail;
 }
 if((flags & HP3478_CMD_LISTEN) == 0) {
  gpib_talk();
  set_atn(1);
  cmd[0] = '_';
  if(!gpib_transmit_b(cmd, 1, 0)) {
   errcode = 9;
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
 if(!hp3478_cmd(cmd, 2, HP3478_CMD_CONT|HP3478_CMD_NO_LF)) {
  L2_ERRCODE(1);
  return 0;
 }
 if(!hp3478_cmd((const uint8_t*)display, len, HP3478_CMD_CONT)) {
  L2_ERRCODE(2);
  return 0;
 }
 if(!hp3478_cmd(0, 0, flags)) {
  L2_ERRCODE(3);
  return 0; /* send additional LF */
 }
 return 1;
}

static uint8_t
hp3478_display_P(const char *display, uint8_t flags)
{
 uint8_t cmd[2];

 cmd[0] = 'D'; /* display */
 cmd[1] = (flags & HP3478_DISP_HIDE_ANNUNCIATORS) != 0 ? '3' : '2';
 if(!hp3478_cmd(cmd, 2, HP3478_CMD_CONT|HP3478_CMD_NO_LF)) {
  L2_ERRCODE(1);
  return 0;
 }
 if(!hp3478_cmd_P(display, HP3478_CMD_CONT)) {
  L2_ERRCODE(2);
  return 0;
 }
 if(!hp3478_cmd(0, 0, flags)) {
  L2_ERRCODE(3);
  return 0; /* send additional LF */
 }
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

 if(!hp3478_read(buf, sizeof(buf), &len, flags)) {
  L2_ERRCODE(4);
  return 0;
 }

 i = 0;
 sign = buf[0] == '-';
 v = 0;
 for(++i; i < len; i++) {
  if(buf[i] == 'E') break;
  if(buf[i] == '.') reading->dot = i-1;
  else v = v*10 + (buf[i]-'0');
 }
 i++;
 if(len-i < 2) {
  printf_P(PSTR("l2err 5 %d %d\n"), len, i);
  L2_ERRCODE(5);
  return 0;
 }
 reading->value = sign ? -v : v;
 sign = buf[i++] == '-';
 reading->exp = sign ? '0'-buf[i] : buf[i]-'0';
 return 1;
}

static uint8_t
hp3478_get_status(uint8_t st[5])
{
 uint8_t rl;
 if(!hp3478_cmd_P(PSTR("B"), HP3478_CMD_TALK)) {
  L2_ERRCODE(6);
  return 0;
 }
 if(!hp3478_read(st, 5, &rl, 0)) {
  L2_ERRCODE(7);
  return 0;
 }
 return rl == 5;
}

#if 0
static uint8_t
hp3478_get_fpsw(uint8_t *c)
{
 uint8_t rl;
 if(!hp3478_cmd_P(PSTR("S"), HP3478_CMD_CONT)) {
  return 0;
 }
 if(!hp3478_read(c, 3, &rl, 0)) {
  return 0;
 }
 return rl == 3;
}
#endif

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
 if(mode_ind == 'b') {
   display[0] = '>';
 } else if(r->value >= 0) { 
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
 if(mode_ind >= 'a') display[i++] = ' ';
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
 if(mode_ind < 'a') display[12] = mode_ind;
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
 if(!hp3478_cmd_P(PSTR("M21T1"), 0)) {
  L2_ERRCODE(8);
  return 0;
 }
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

 if(!hp3478_display_reading(&out, hp3478_rel_mode, '*', 0)) {
  L3_ERRCODE(30);
  return 0;
 }
 return 1;
}

static uint8_t hp3478_menu_timeout;
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
#define HP3478_MENU_XOHM_DIODE 14
#define HP3478_MENU_PRESET 15
/*#define HP3478_MENU_PRESET_LOAD 16*/
#define HP3478_MENU_PRESET_LOAD0 17
#define HP3478_MENU_PRESET_LOAD1 18
#define HP3478_MENU_PRESET_LOAD2 19
#define HP3478_MENU_PRESET_LOAD3 20
#define HP3478_MENU_PRESET_LOAD4 21
#define HP3478_MENU_PRESET_SAVE 22
#define HP3478_MENU_PRESET_SAVE0 23
#define HP3478_MENU_PRESET_SAVE1 24
#define HP3478_MENU_PRESET_SAVE2 25
#define HP3478_MENU_PRESET_SAVE3 26
#define HP3478_MENU_PRESET_SAVE4 27

static uint8_t hp3478_btn_detect_stage;

static uint8_t 
hp3478_menu_next(uint8_t pos)
{
 switch(pos) {
         case HP3478_MENU_XOHM_BEEP:
                 return HP3478_MENU_XOHM;
         case HP3478_MENU_XOHM:
                 return HP3478_MENU_XOHM_DIODE;
         case HP3478_MENU_BEEP:
                 return HP3478_MENU_DIODE;
         case HP3478_MENU_XOHM_DIODE:
                 return HP3478_MENU_AUTOHOLD;
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
                 return HP3478_MENU_PRESET;
         case HP3478_MENU_PRESET:
         case HP3478_MENU_PRESET_SAVE4:
         case HP3478_MENU_PRESET_LOAD4:
                 break;
         /*case HP3478_MENU_PRESET_LOAD:
                 return HP3478_MENU_PRESET_SAVE;*/
         case HP3478_MENU_PRESET_SAVE:
                 return HP3478_MENU_PRESET_LOAD0;
         case HP3478_MENU_PRESET_LOAD0:
                 return HP3478_MENU_PRESET_LOAD1;
         case HP3478_MENU_PRESET_LOAD1:
                 return HP3478_MENU_PRESET_LOAD2;
         case HP3478_MENU_PRESET_LOAD2:
                 return HP3478_MENU_PRESET_LOAD3;
         case HP3478_MENU_PRESET_LOAD3:
                 return HP3478_MENU_PRESET_LOAD4;
         case HP3478_MENU_PRESET_SAVE0:
                 return HP3478_MENU_PRESET_SAVE1;
         case HP3478_MENU_PRESET_SAVE1:
                 return HP3478_MENU_PRESET_SAVE2;
         case HP3478_MENU_PRESET_SAVE2:
                 return HP3478_MENU_PRESET_SAVE3;
         case HP3478_MENU_PRESET_SAVE3:
                 return HP3478_MENU_PRESET_SAVE4;
 }
 return HP3478_MENU_DONE;
}

static uint8_t
hp3478_menu_show(uint8_t pos)
{
 const char *s;
 switch(pos) {
         case HP3478_MENU_OHM_MINMAX:
         case HP3478_MENU_MINMAX: s = PSTR("M: MINMAX"); break;
         case HP3478_MENU_XOHM_BEEP:
         case HP3478_MENU_BEEP: s = PSTR("M: CONT"); break;
         case HP3478_MENU_XOHM: s = PSTR("M: XOHM"); break;
         case HP3478_MENU_OHM_AUTOHOLD:
         case HP3478_MENU_AUTOHOLD: s = PSTR("M: AUTOHOLD"); break;
         case HP3478_MENU_XOHM_DIODE:
         case HP3478_MENU_DIODE: s = PSTR("M: DIODE"); break;
         case HP3478_MENU_TEMP: s = PSTR("M: TEMP"); break;
         case HP3478_MENU_PRESET: s = PSTR("M: PRESET"); break;
         case HP3478_MENU_PRESET_SAVE: s = PSTR("P: SAVE"); break;
         case HP3478_MENU_PRESET_SAVE0: s = PSTR("S: SAVE0"); break;
         case HP3478_MENU_PRESET_SAVE1: s = PSTR("S: SAVE1"); break;
         case HP3478_MENU_PRESET_SAVE2: s = PSTR("S: SAVE2"); break;
         case HP3478_MENU_PRESET_SAVE3: s = PSTR("S: SAVE3"); break;
         case HP3478_MENU_PRESET_SAVE4: s = PSTR("S: SAVE4"); break;
         /*case HP3478_MENU_PRESET_LOAD: s = PSTR("P: LOAD"); break;*/
         case HP3478_MENU_PRESET_LOAD0: s = PSTR("L: LOAD0"); break;
         case HP3478_MENU_PRESET_LOAD1: s = PSTR("L: LOAD1"); break;
         case HP3478_MENU_PRESET_LOAD2: s = PSTR("L: LOAD2"); break;
         case HP3478_MENU_PRESET_LOAD3: s = PSTR("L: LOAD3"); break;
         case HP3478_MENU_PRESET_LOAD4: s = PSTR("L: LOAD4"); break;
 }
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
 if(!hp3478_display_P(s, HP3478_DISP_HIDE_ANNUNCIATORS|HP3478_CMD_CONT)) return 0;
#pragma GCC diagnostic pop
 return 1;
}

static uint8_t 
hp3478_menu_restart_btn_detect(void)
{
 /* invalid command to trigger HP3478_SB_SYNERR */
 if(!hp3478_cmd_P(PSTR("A"), HP3478_CMD_REMOTE|HP3478_CMD_TALK)) {
  L2_ERRCODE(9);
  return 0;
 }
 hp3478_btn_detect_stage = 0;
 return 1;
}

static uint8_t 
hp3478_submenu_init(uint8_t pos)
{
 hp3478_menu_timeout = 0;
 hp3478_menu_pos = pos;
 if(!hp3478_menu_show(hp3478_menu_pos)) {
  L3_ERRCODE(60);
  return 0;
 }
 if(!hp3478_menu_restart_btn_detect()) {
  L3_ERRCODE(61);
  return 0;
 }
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
                  if(!hp3478_cmd_P(PSTR("M24"), HP3478_CMD_REMOTE|HP3478_CMD_TALK)) {
                   L3_ERRCODE(50);
                   return HP3478_MENU_ERROR;
                  }
                  return HP3478_MENU_WAIT;
                 }
                 return HP3478_MENU_NOP;
         case 1:
                 if((ev & (EV_TIMEOUT|EV_SRQ)) != 0 && !srq()) break;
                 if((ev & EV_TIMEOUT) != 0) {
                  hp3478_btn_detect_stage = 0;
                  if(!hp3478_cmd_P(PSTR("M20"), HP3478_CMD_REMOTE|HP3478_CMD_TALK)) {
                   L3_ERRCODE(51);
                   return HP3478_MENU_ERROR;
                  }
                  return HP3478_MENU_WAIT;
                 }
                 return HP3478_MENU_NOP;
 }
 hp3478_menu_timeout = 0;
 if(!hp3478_get_srq_status(&sb)){
  L3_ERRCODE(52);
  return HP3478_MENU_ERROR;
 }
 if(!hp3478_cmd_P(PSTR("KM20"), 0)) {
  L3_ERRCODE(53);
  return HP3478_MENU_ERROR;
 }
 if(sb & HP3478_SB_FRPSRQ) {
  hp3478_menu_pos = hp3478_menu_next(hp3478_menu_pos);
  if(hp3478_menu_pos == HP3478_MENU_DONE) return HP3478_MENU_DONE;
  if(!hp3478_menu_show(hp3478_menu_pos)) {
   L3_ERRCODE(54);
   return HP3478_MENU_ERROR;
  }
 } else {
  /* assume Local button pressed */
  return hp3478_menu_pos;
 }
                  
 if(!hp3478_menu_restart_btn_detect()) {
  L3_ERRCODE(2);
  return HP3478_MENU_ERROR;
 }
 return HP3478_MENU_WAIT;
}

static uint32_t hp3478_xohm_10M;

static uint8_t 
hp3478_xohm_init(void)
{
 hp3478_xohm_10M = 0;
 if(!hp3478_cmd_P(PSTR("F7M21"), 0)) {
  L2_ERRCODE(10);
  return 0; // FIXME: F7N5M21?
 }
 return 1;
}

static uint8_t 
hp3478_xohm_handle_data(struct hp3478_reading *reading)
{
 uint32_t r, n;
 struct hp3478_reading rr = *reading;

 if(hp3478_xohm_10M == 0) hp3478_xohm_10M = rr.value;

 if(hp3478_xohm_10M <= rr.value + 100)  {
  if(!hp3478_display_P(PSTR("  OVLD  GOHM"), 0)) {
   L4_ERRCODE(4);
   return 0;
  }
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
 if(!hp3478_display_reading(&rr, HP3478_ST_FUNC_2WOHM|HP3478_ST_N_DIGITS5, 'z', 0)) {
  L3_ERRCODE(3);
  return 0;
 }
 return 1;
}

static uint8_t minmax_state = 0;
static uint8_t 
hp3478_diode_init(void)
{
 uint8_t s[5];
 if(!hp3478_get_status(s)) {
  L3_ERRCODE(4);
  return 0;
 }
 hp3478_saved_state[0] = s[0];
 hp3478_saved_state[1] = s[1];
 if(!hp3478_cmd_P(PSTR("R3M21"), 0)) {
  L3_ERRCODE(5);
  return 0;
 }
 minmax_state = 1;
 return 1;
}

static uint8_t 
hp3478_diode_handle_data(struct hp3478_reading *reading)
{
 if(reading->exp == 9) {
  if(minmax_state) {
   minmax_state = 0;
   if(!hp3478_display_P(PSTR("     >3 V"), HP3478_DISP_HIDE_ANNUNCIATORS)) {
    L3_ERRCODE(6);
    return 0;
   }
  }
  return 1;
 }
 minmax_state = 1;
 reading->exp = 0;
 if(!hp3478_display_reading(reading, hp3478_saved_state[0], 'd', 0)) {
  L3_ERRCODE(7);
  return 0;
 }
 return 1;
}

static uint8_t 
hp3478_temp_init(void)
{
 uint8_t s[5];
 if(!hp3478_get_status(s)) {
  L3_ERRCODE(8);
  return 0;
 }
 hp3478_saved_state[0] = s[0];
 /* hp3478_saved_state[1] = s[1]; */
 if(!hp3478_cmd_P(PSTR("M21"), 0)) {
  L3_ERRCODE(9);
  return 0;
 }
 minmax_state = 1;
 return 1;
}

static uint8_t 
hp3478_temp_handle_data(struct hp3478_reading *reading)
{
 if(reading->exp == 9) {
  if(minmax_state) {
   minmax_state = 0;
   if(!hp3478_display_P(PSTR("  OPEN"), HP3478_DISP_HIDE_ANNUNCIATORS)) {
    L3_ERRCODE(10);
    return 0;
   }
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
 if(!hp3478_display_reading(reading, hp3478_saved_state[0], 'c', 0)) {
  L3_ERRCODE(11);
 }
 return 0; 
}

static uint8_t 
hp3478_set_mode(uint8_t s1, uint8_t s2)
{
 uint8_t cmd[11]; /* R__N_F_Z_T_ */
 uint8_t p = 0;
 uint8_t func;
 uint8_t range;

 func = s1 & HP3478_ST_FUNC;
 range = s1 & HP3478_ST_RANGE;

 cmd[p++] = 'R';

 if(s2 & HP3478_ST_AUTORANGE) cmd[p++] = 'A';
 else if(range == 0) cmd[p++] = 'A';
 else switch(func) {
         case HP3478_ST_FUNC_XOHM: 
         case HP3478_ST_FUNC_2WOHM:
         case HP3478_ST_FUNC_4WOHM:
                 cmd[p++] = '0' + (range>>2);
                 break;
         case HP3478_ST_FUNC_ACA:
         case HP3478_ST_FUNC_DCA:
                 if(range == HP3478_ST_RANGE1) {cmd[p++] = '-'; cmd[p++] = '1';}
                 else cmd[p++] = '0';
                 break;
         case HP3478_ST_FUNC_ACV:
                 if(range == HP3478_ST_RANGE1) {cmd[p++] = '-'; cmd[p++] = '1';}
                 else cmd[p++] = '0' - 1 + (range>>2);
                 break;
         default: /* DCV */
                 if(range < HP3478_ST_RANGE3) {cmd[p++] = '-'; cmd[p++] = '0' + 3 - (range>>2);}
                 else cmd[p++] = '0' - 3 + (range>>2);
 }
 cmd[p++] = 'N';
 switch(s1 & HP3478_ST_N_DIGITS) {
         case HP3478_ST_N_DIGITS4: cmd[p++] = '4'; break;
         case HP3478_ST_N_DIGITS3: cmd[p++] = '3'; break;
         default: cmd[p++] = '5';
 }

 cmd[p++] = 'F';
 switch(func) {
         case 0: cmd[p++] = '1'; break; /* invalid value */
         case HP3478_ST_FUNC_XOHM: cmd[p++] = '3'; break;
         default: cmd[p++] = '0' + (func>>5);
 }

 cmd[p++] = 'Z';
 if(s2 & HP3478_ST_AUTOZERO) cmd[p++] = '1';
 else cmd[p++] = '0';

 cmd[p++] = 'T';
 if(s2 & HP3478_ST_INT_TRIGGER) cmd[p++] = '1';
 else if(s2 & HP3478_ST_EXT_TRIGGER) cmd[p++] = '2';
 else cmd[p++] = '3';

 if(!hp3478_cmd(cmd, p, 0)) {
  L2_ERRCODE(11);
  return 0;
 }
 return 1;
}

/* TODO: replace cont_fini with set_mode */
static uint8_t 
hp3478_cont_fini(void)
{
 uint8_t s1, s2;
 uint8_t cmd[6];

 s1 = hp3478_saved_state[0];
 s2 = hp3478_saved_state[1];
 beep_off();
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
 if(!hp3478_cmd(cmd, sizeof(cmd), 0)) {
  L2_ERRCODE(11);
  return 0;
 }
 return 1;
}


/* TODO: extend this table for all modes and use it in hp3478_display_reading */
const uint8_t hp3478_cont_range2exp[7] PROGMEM = {2, 3, 3<<4|1, 3<<4|2, 3<<4|3, 6<<4|1, 6<<4|2}; 

static int
hp3478_cont_show_thres(void)
{
 struct hp3478_reading r;
 uint8_t dot_exp;

 dot_exp = pgm_read_byte(hp3478_cont_range2exp + cont_range);
 r.value = (uint32_t)cont_threshold*100;
 r.dot = dot_exp & 0x0f;
 r.exp = dot_exp >> 4;

 if(!hp3478_display_reading(&r, HP3478_ST_N_DIGITS3|HP3478_ST_FUNC_2WOHM, 'b', HP3478_DISP_HIDE_ANNUNCIATORS)) {
  L3_ERRCODE(12);
  return 0;
 }
 return 1;
}

static uint8_t 
hp3478_cont_init(void)
{
 uint8_t s[5];
 if(!hp3478_get_status(s)) {
  L3_ERRCODE(13);
  return 0;
 }
 hp3478_saved_state[0] = s[0];
 hp3478_saved_state[1] = s[1];
 s[0] = 'R';
 s[1] = '1'+cont_range;
 if(!hp3478_cmd(s, 2, HP3478_CMD_NO_LF)) {
  L3_ERRCODE(14);
  return 0;
 }
 if(!hp3478_cmd_P(PSTR("N3M21Z0"), 0)) {
  L3_ERRCODE(15);
  return 0;
 }
 if(!hp3478_cont_show_thres()) return 0;
 cont_latch_dncnt = 0;
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
 if(!hp3478_get_status(s)) {
  L3_ERRCODE(16);
  return 0;
 }
 hp3478_saved_state[0] = s[0];
 if(!hp3478_cmd_P(PSTR("M21"), 0)) {
  L3_ERRCODE(17);
  return 0;
 }
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
 int8_t res_sign = 1;

 if(rr1 < 0 && rr2 >= 0) return -1;
 if(rr2 < 0 && rr1 >= 0) return 1;

 if(e1 < e2) {
  uint8_t e_tmp;
  int32_t rr_tmp;
  e_tmp = e1; e1 = e2; e2 = e_tmp;
  rr_tmp = rr1; rr1 = rr2; rr2 = rr_tmp;
  res_sign = -res_sign;
 }

 if(rr1 >= 0) 
  while(1) {
   if(rr1 > rr2) return res_sign;
   if(e1 == e2) {
    if(rr1 == rr2) return 0;
    return -res_sign;
   }
   rr1 *= 10;
   e1--;
  }

 while(1) {
  if(rr1 < rr2) return -res_sign;
  if(e1 == e2) {
   if(rr1 == rr2) return 0;
   return res_sign;
  }
  rr1 *= 10;
  e1--;
 }
}

static uint8_t 
hp3478_minmax_detect_key(void)
{
 if(!srq()) {
  uint8_t s[5];

  if(!hp3478_get_status(s)) return 1; /* it normally fails if 3478A is in LOCAL */
  if((s[2] & HP3478_SB_DREADY) == 0) {
    return 1; /* Sometimes LOCAL key prevents enabling DREADY SRQ.
                 Assume LOCAL is pressed if DREADY is not enabled here. */
  }
  return 0;
 }
 if(!hp3478_cmd_P(PSTR("M20"), HP3478_CMD_CONT)) { /* TODO: alternatively we can toggle SYNERR bit and check if 
                                                            it's reflected in the status byte */
  printf_P(PSTR("M20 failed\r\n")); /* TODO: errcode */
  return 1;
 }
 _delay_us(400); /* ~250 uS it takes to clear the SRQ after mask update */
 if(srq()) return 1; /* It's either FPSRQ or M20 command was not accepted (due to LOCAL). */
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
                  if(!hp3478_display_P(PSTR("NO MIN"), HP3478_CMD_CONT|HP3478_DISP_HIDE_ANNUNCIATORS)) {
                   L3_ERRCODE(18);
                   return 0;
                  }
                 } else {
                  d = minmax_min;
                  if(!hp3478_display_reading(&d, hp3478_saved_state[0], '-', HP3478_CMD_CONT|HP3478_DISP_HIDE_ANNUNCIATORS)) {
                   L3_ERRCODE(19);
                   return 0;
                  }
                 }
                 return 1;
         case MINMAX_DISP_MIN:
                 if(!key_press) {
                  if((r & MINMAX_MIN) == 0) break;
                  d = minmax_min;
                  if(!hp3478_display_reading(&d, hp3478_saved_state[0], '-', HP3478_CMD_CONT|HP3478_DISP_HIDE_ANNUNCIATORS)) {
                   L3_ERRCODE(20);
                   return 0;
                  }
                  return 1;
                 }
                 minmax_state = (s & ~MINMAX_DISP) | MINMAX_DISP_MAX;
                 if((s & MINMAX_MAX) == 0) {
                  if(!hp3478_display_P(PSTR("NO MAX"), HP3478_CMD_CONT|HP3478_DISP_HIDE_ANNUNCIATORS)) {
                   L3_ERRCODE(21);
                   return 0;
                  }
                 } else {
                  d = minmax_max;
                  if(!hp3478_display_reading(&d, hp3478_saved_state[0], '+', HP3478_CMD_CONT|HP3478_DISP_HIDE_ANNUNCIATORS)) {
                   L3_ERRCODE(22);
                   return 0;
                  }
                 }
                 return 1;
         case MINMAX_DISP_MAX:
                 if(!key_press) {
                  if((r & MINMAX_MAX) == 0) break;
                  d = minmax_max;
                  if(!hp3478_display_reading(&d, hp3478_saved_state[0], '+', HP3478_CMD_CONT|HP3478_DISP_HIDE_ANNUNCIATORS)) {
                   L3_ERRCODE(23);
                   return 0;
                  }
                  return 1;
                 }
                 minmax_state = s & ~MINMAX_DISP;
                 if(!hp3478_cmd_P(PSTR("D1"), HP3478_CMD_CONT)) {
                  L3_ERRCODE(24);
                  return 0;
                 }
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
 if(!hp3478_get_status(s)) {
  L3_ERRCODE(25);
  return 0;
 }
 hp3478_saved_state[0] = s[0];
 hp3478_saved_state[1] = s[1];
 if(!hp3478_cmd_P(PSTR("M21T1"), 0)) {
  L3_ERRCODE(26);
  return 0;
 }
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
 if(!hp3478_get_reading(&r, HP3478_CMD_CONT)) {
  L3_ERRCODE(27);
  return AHLD_ERROR;
 }

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
   if(!hp3478_cmd_P(PSTR("T4"), 0)) {
    L3_ERRCODE(28);
    return AHLD_ERROR;
   }
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

 if(!hp3478_display_reading(&r, st, '?', 0)) {
  L3_ERRCODE(29);
  return AHLD_ERROR;
 }
 return ret;
}

#define HP3478_REINIT do { \
 state = HP3478_INIT; \
 return 250; \
} while(0)

#define HP3478_REINIT_ERR(C) do { \
 state = HP3478_INIT; \
 L4_ERRCODE(C); \
 return 250; \
} while(0)

static char 
hex_digit(uint8_t d)
{
 return d < 10 ? d + '0' : d + ('A' - 10);
}

static uint8_t
hp3478_display_err(void)
{
 char buf[sizeof("E:XXXXXXXX")] = "E:";
 buf[2] = hex_digit(errcode4>>4);
 buf[3] = hex_digit(errcode4&15);
 buf[4] = hex_digit(errcode3>>4);
 buf[5] = hex_digit(errcode3&15);
 buf[6] = hex_digit(errcode2>>4);
 buf[7] = hex_digit(errcode2&15);
 buf[8] = hex_digit(errcode>>4);
 buf[9] = hex_digit(errcode&15);
 return hp3478_display(buf, 10, 0);
}

static void
preset_save(uint8_t num, uint8_t st[5])
{
 struct opt_info o;
 uint16_t s;
 uint8_t include = 0;
 uint8_t i;
 /* TODO: save ext function if menu is activated within 5 sec after exitting ext function */

 s = (uint16_t)st[0]+((uint16_t)st[1]<<8);
 if(num == 0) hp3478_init_mode = s;
 eeprom_write_word((uint16_t*)(EEP_ADDR_MODE+num*EEP_PRESET_SIZE), s);

 for(i = 0; i < sizeof(opts)/sizeof(opts[0]); i++) {
  memcpy_P(&o, opts+i, sizeof(*opts));
  if(o.addr_eep == (void*)EEP_ADDR_BEEP_PERIOD) include = 1;
  if(!include) continue;
  if(o.flags & OPT_INFO_W16) eeprom_write_word(o.addr_eep, *(uint16_t*)o.addr);
  else eeprom_write_byte(o.addr_eep, *(uint8_t*)o.addr);
 }
}

static uint8_t
preset_load(uint8_t num)
{
 struct opt_info o;
 uint16_t val;
 uint8_t st1, st2;
 uint8_t i, include;

 val = eeprom_read_word((uint16_t*)(EEP_ADDR_MODE+num*EEP_PRESET_SIZE));
 st1 = val&0xff;
 st2 = val>>8;
 if((st1 & HP3478_ST_FUNC) == 0 || (st1 & HP3478_ST_RANGE) == 0 || (st1 & HP3478_ST_N_DIGITS) == 0
     || (st2 & 0x80) != 0) 
  return hp3478_display_P(PSTR("BAD PRESET"), 0);
 hp3478_init_mode = val;
 
 for(i = 0; i < sizeof(opts)/sizeof(opts[0]); i++) {
  memcpy_P(&o, opts+i, sizeof(*opts));
  if(o.addr_eep == (void*)EEP_ADDR_BEEP_PERIOD) include = 1;
  if(!include) continue;
  if(o.flags & OPT_INFO_W16) { /* TODO: merge with load_settings */
   val = eeprom_read_word(o.addr_eep);
   if(val <= o.max) *(uint16_t*)o.addr = val;
  } else {
   val = eeprom_read_byte(o.addr_eep);
   if(val <= o.max) *(uint8_t*)o.addr = val;
  }
 }
 return hp3478_set_mode(st1, st2);
}

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
 uint8_t menu_pos;

 if(state == HP3478_DISA) {
  if((ev & EV_EXT_ENABLE) == 0) return TIMEOUT_INF;
  state = HP3478_INIT;
 }

 if((ev & EV_EXT_DISABLE) != 0) {
  switch(state) {
          case HP3478_AHLL:
          case HP3478_AHLD:
                  beep_off();
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

 if(sb & HP3478_SB_PWRSRQ) {
  if(hp3478_init_mode) 
   hp3478_set_mode(hp3478_init_mode & 0xff, hp3478_init_mode >> 8);
 }
 if(state != HP3478_INIT && state != HP3478_MENU && state != HP3478_MMAX) {
  if(!hp3478_get_srq_status(&sb)) HP3478_REINIT_ERR(5);
  if(sb & HP3478_SB_PWRSRQ) {
   if(hp3478_init_mode) 
    hp3478_set_mode(hp3478_init_mode & 0xff, hp3478_init_mode >> 8);
   HP3478_REINIT;
  }
  if(sb & HP3478_SB_FRPSRQ) {
   switch(state) {
          case HP3478_AHLL:
          case HP3478_AHLD:
                  beep_off();
                  if(!hp3478_cmd_P(PSTR("KM20D1T1"), 0)) HP3478_REINIT_ERR(6);
                  break;

          case HP3478_IDLE:
                 if((sb & HP3478_SB_DREADY) != 0)
                  if(!hp3478_get_reading(&reading, HP3478_CMD_LISTEN)) HP3478_REINIT_ERR(7);

                 /* K is required, because serial poll doesn't clear status bits
                    immediately. Otherwise next SRQ still may be seen as FP SRQ. */  
                 if(!hp3478_cmd_P(PSTR("K"), HP3478_CMD_CONT)) HP3478_REINIT_ERR(8);
                 if(!hp3478_get_status(st)) HP3478_REINIT_ERR(9);
                 if((st[1] & HP3478_ST_INT_TRIGGER) == 0) {
                  if((sb & HP3478_SB_DREADY) == 0) {
                   if(!hp3478_cmd_P(PSTR("M21"), 0)) HP3478_REINIT_ERR(10);
                   state = HP3478_RELS;
                   return 1800;
                  }
                  if(reading.exp == 9) {
                   if(!hp3478_autohold_init()) HP3478_REINIT_ERR(11);
                   state = HP3478_AHLD;
                   return 0xffff;
                  }
                          
                  if(!hp3478_rel_start(st[0], &reading)) HP3478_REINIT_ERR(12);
                  state = HP3478_RELA;
                  return 0xffff;
                 }

                 {
                  uint8_t p;

                  if((st[0] & HP3478_ST_FUNC) == HP3478_ST_FUNC_2WOHM) {
                   if(reading.exp == 9) p = HP3478_MENU_XOHM_BEEP;
                   else p = HP3478_MENU_BEEP;
                  } else  if((st[0] & HP3478_ST_FUNC) == HP3478_ST_FUNC_XOHM) {
                   p = HP3478_MENU_XOHM;
                  } else p = HP3478_MENU_AUTOHOLD;
                  if(!hp3478_submenu_init(p)) HP3478_REINIT_ERR(44);
                 }
                 state = HP3478_MENU;
                 return 100;

          case HP3478_CONT:
          case HP3478_DIOD:
                  if(!hp3478_cont_fini()) HP3478_REINIT_ERR(13);

          default:
                  if(!hp3478_cmd_P(PSTR("KM20D1"), 0)) HP3478_REINIT_ERR(14);
   }
   state = HP3478_IDLE;
   return TIMEOUT_INF;
  }
 }

 switch(state) {
         case HP3478_INIT:
                 if(!hp3478_get_srq_status(&sb)) return 2000; /* retry initialization after 2 sec */
                 if(sb & HP3478_SB_PWRSRQ) {
                  if(hp3478_init_mode) 
                   hp3478_set_mode(hp3478_init_mode & 0xff, hp3478_init_mode >> 8);
                 }
                 if(!hp3478_cmd_P(PSTR("KM20"), 0)) return 2000;
                 printf_P(PSTR("init: ok\r\n"));
                 if(errcode|errcode2|errcode3|errcode4) {
                    if(!hp3478_display_err()) return 2000;
                    errcode = 0;
                    errcode2 = 0;
                    errcode3 = 0;
                    errcode4 = 0;
                 }
                 state = HP3478_IDLE;
                 return TIMEOUT_INF;

         case HP3478_IDLE:
                 if(!hp3478_cmd_P(PSTR("K"), 0)) HP3478_REINIT_ERR(15);
                 printf_P(PSTR("idle: unexpected ev %x %x\r\n"), (unsigned)ev, (unsigned)sb);
                 return TIMEOUT_INF;

         case HP3478_MENU:
                 switch(menu_pos = hp3478_menu_process(ev)) {
                         default:
                                                 printf_P(PSTR("menu: unknown\r\n"));
                         case HP3478_MENU_ERROR: 
                                                 HP3478_REINIT;
                         case HP3478_MENU_BEEP: 
                         case HP3478_MENU_XOHM_BEEP: 
                                                 state = HP3478_CONT;
                                                 if(!hp3478_cont_init()) HP3478_REINIT_ERR(16);
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
                                                 if(!hp3478_autohold_init()) HP3478_REINIT_ERR(17);
                                                 return 0xffff;
                         case HP3478_MENU_XOHM_DIODE: 
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
                         case HP3478_MENU_PRESET_SAVE0: 
                         case HP3478_MENU_PRESET_SAVE1: 
                         case HP3478_MENU_PRESET_SAVE2: 
                         case HP3478_MENU_PRESET_SAVE3: 
                         case HP3478_MENU_PRESET_SAVE4: 
                                                 if(!hp3478_get_status(st)) HP3478_REINIT_ERR(45);
                                                 preset_save(menu_pos-HP3478_MENU_PRESET_SAVE0, st);
                                                 state = HP3478_IDLE;
                                                 return TIMEOUT_INF;
                         case HP3478_MENU_PRESET_LOAD0: 
                         case HP3478_MENU_PRESET_LOAD1: 
                         case HP3478_MENU_PRESET_LOAD2: 
                         case HP3478_MENU_PRESET_LOAD3: 
                         case HP3478_MENU_PRESET_LOAD4: 
                                                 if(!preset_load(menu_pos-HP3478_MENU_PRESET_LOAD0)) HP3478_REINIT_ERR(46);
                                                 state = HP3478_IDLE;
                                                 return TIMEOUT_INF;
                         case HP3478_MENU_NOP: 
                                                 return TIMEOUT_CONT;
                         case HP3478_MENU_WAIT: 
                                                 if(++hp3478_menu_timeout == 300) { /* 30 sec */
                                                  state = HP3478_IDLE;
                                                  if(!hp3478_cmd_P(PSTR("D1KM20"), 0)) HP3478_REINIT_ERR(40);
                                                  printf_P(PSTR("menu: timeout\r\n"));
                                                  return TIMEOUT_INF;
                                                 }
                                                 return 100;
                         case HP3478_MENU_PRESET:
                                                 if(!hp3478_submenu_init(HP3478_MENU_PRESET_SAVE))
                                                  HP3478_REINIT_ERR(41);
                                                 return 100;
                         case HP3478_MENU_PRESET_SAVE:
                                                 if(!hp3478_submenu_init(HP3478_MENU_PRESET_SAVE0))
                                                  HP3478_REINIT_ERR(42);
                                                 return 100;
                         /*case HP3478_MENU_PRESET_LOAD:
                                                 if(!hp3478_submenu_init(HP3478_MENU_PRESET_LOAD0))
                                                  HP3478_REINIT_ERR(43);
                                                 return 100;*/
                 }


         case HP3478_RELS:
                 if((ev & EV_TIMEOUT) != 0) {
                  if(!hp3478_autohold_init()) HP3478_REINIT_ERR(18);
                  state = HP3478_AHLD;
                  return 0xffff;
                 }
                 if((sb & HP3478_SB_DREADY) == 0) return TIMEOUT_CONT;
                 if(!hp3478_get_reading(&reading, HP3478_CMD_LISTEN)) HP3478_REINIT_ERR(19);
                 if(reading.exp == 9) {
                  if(!hp3478_autohold_init()) HP3478_REINIT_ERR(20);
                  state = HP3478_AHLD;
                  return 0xffff;
                 }
                 if(!hp3478_get_status(st)) HP3478_REINIT_ERR(21);
                 if(!hp3478_rel_start(st[0], &reading)) HP3478_REINIT_ERR(22);
                 state = HP3478_RELA;
                 return 0xffff;

         case HP3478_AHLD:
         case HP3478_AHLL:
                 switch(hp3478_autohold_process(state == HP3478_AHLL, sb)) {
                         case AHLD_ERROR:        beep_off();
                                                 HP3478_REINIT;
                         case AHLD_LOCK: 
                                                 beep(buzz_period, buzz_duty);
                                                 state = HP3478_AHLL;
                                                 return 300;
                         case AHLD_UNLOCK: 
                                                 state = HP3478_AHLD;
                                                 beep_off();
                                                 return TIMEOUT_INF;
                         default:
                                                 if(state == HP3478_AHLL) {
                                                  if(ev & EV_TIMEOUT) {
                                                   beep_off();
                                                   return TIMEOUT_INF;
                                                  }
                                                  return TIMEOUT_CONT;
                                                 }
                                                 return TIMEOUT_INF;
                 }
         case HP3478_RELA:
                  // TODO: also detect Local button?
                 if(sb & HP3478_SB_DREADY) {
                  if(!hp3478_get_reading(&reading, HP3478_CMD_LISTEN)) HP3478_REINIT_ERR(23);
                  if(!hp3478_rel_handle_data(&reading)) { //TODO: pass status bytes, so it knows that nothing's changed
                   HP3478_REINIT;
                  }
                  return 0xffff;
                 } 
                 return 0xffff; /* what was it? */
         case HP3478_TEMP:
                if(sb & HP3478_SB_DREADY) {
                  if(!hp3478_get_reading(&reading, HP3478_CMD_LISTEN)) HP3478_REINIT_ERR(24);
                  if(!hp3478_cmd_P(PSTR("K"), HP3478_CMD_CONT)) HP3478_REINIT_ERR(25);
                  if(!hp3478_temp_handle_data(&reading)) HP3478_REINIT;
                  return 0xffff;
                }
                return 0xffff;
         case HP3478_XOHM:
                if(sb & HP3478_SB_DREADY) {
                  if(!hp3478_get_reading(&reading, HP3478_CMD_LISTEN)) HP3478_REINIT_ERR(26);
                  if(!hp3478_cmd_P(PSTR("K"), HP3478_CMD_CONT)) HP3478_REINIT_ERR(27);
                  if(!hp3478_xohm_handle_data(&reading)) HP3478_REINIT;
                  return 0xffff;
                }
                return 0xffff;
         case HP3478_CONT:
                if(sb & HP3478_SB_DREADY) {
                  if(!hp3478_get_reading(&reading, HP3478_CMD_LISTEN)) HP3478_REINIT_ERR(28);
                  /* uart_tx('r'); */
                  if(reading.value < cont_threshold*100) {
                   if(!cont_latch_dncnt) {
                    if(!hp3478_cmd_P(PSTR("D1"), 0)) HP3478_REINIT_ERR(29);
                   }
                   cont_beep((uint16_t)(reading.value/100));
                   cont_latch_dncnt = cont_latch;
                  } else if(buzzer) {
                   if(cont_latch_dncnt) cont_latch_dncnt--;
                   else {
                    if(!hp3478_cont_show_thres()) HP3478_REINIT_ERR(30);
                    beep_off();
                   }
                  }
                  return 2; /* Come back shortly to check if the mode is not changed:
                               this delay is chosen so not to interrupt 3478 when it's doing
                               something and keep reading rate at maximum. 
                               The expected reading rate is 78 rdg/sec @50Hz. */
                }
                if(!hp3478_get_status(st)) HP3478_REINIT_ERR(31);
                if(st[0] != ((cont_range+1)<<2|HP3478_ST_N_DIGITS3|HP3478_ST_FUNC_2WOHM)
                     || (st[1]&7) != HP3478_ST_INT_TRIGGER) {
                   if(!hp3478_cont_fini()) HP3478_REINIT_ERR(32);
                   if(!hp3478_cmd_P(PSTR("M20D1"), 0)) HP3478_REINIT_ERR(33);
                   state = HP3478_IDLE;
                }
                return TIMEOUT_INF;
         case HP3478_DIOD:
                if(sb & HP3478_SB_DREADY) {
                  if(!hp3478_get_reading(&reading, HP3478_CMD_LISTEN)) HP3478_REINIT_ERR(34);
                  if(!hp3478_diode_handle_data(&reading)) HP3478_REINIT_ERR(35);
                }
                return TIMEOUT_INF;
         case HP3478_MMAX:
                {
                 uint8_t minmax_ev;
                 uint8_t k;
                 k = hp3478_minmax_detect_key();

                 if(!hp3478_get_srq_status(&sb)) HP3478_REINIT_ERR(36);
                 if(sb & HP3478_SB_FRPSRQ) _delay_us(250); /* Wait for FPSRQ to be cleared to prevent double detection.
                                                              It seems that 3478A can't keep up with us, and needs
                                                              a break to do it's housekeeping. */
                 if(k && (sb&HP3478_SB_FRPSRQ) == 0) {
                   if(!hp3478_cmd_P(PSTR("KM20D1"), 0)) HP3478_REINIT_ERR(37);
                   state = HP3478_IDLE;
                   return 0xffff;
                 }
                 minmax_ev = 0;
                 if(sb & HP3478_SB_DREADY) {
                   if(!hp3478_get_reading(&reading, HP3478_CMD_CONT)) HP3478_REINIT_ERR(38);
                   minmax_ev = hp3478_minmax_handle_data(&reading);
                 }
                 if(!hp3478_minmax_display_data(minmax_ev, sb&HP3478_SB_FRPSRQ)) HP3478_REINIT;
                 if(!hp3478_cmd_P(PSTR("M21"), HP3478_CMD_CONT)) HP3478_REINIT_ERR(39); /* restore mask after minmax_detect_key */
                }

                return 400; /* in case the LOCAL pressed just before the M21 command, wake up and detect it */
 }
 return 0xffff;
}

static void
set_defaults(uint8_t set)
{
 uint8_t i;
 struct opt_info o;
 for(i = 0; i < sizeof(opts)/sizeof(opts[0]); i++) {
  memcpy_P(&o, opts+i, sizeof(*opts));
  if(o.flags & OPT_INFO_W16) *(uint16_t*)o.addr = o.def;
  else *(uint8_t*)o.addr = o.def;
 }
 /* this is default: hp3478_ext_enable = 0; */
 uart_echo = set == 0;
}

static void
load_settings(void)
{
 uint8_t i;
 uint16_t val;
 struct opt_info o;

 for(i = 0; i < sizeof(opts)/sizeof(opts[0]); i++) {
  memcpy_P(&o, opts+i, sizeof(*opts));
  if(o.flags & OPT_INFO_W16) {
   val = eeprom_read_word(o.addr_eep);
   if(val <= o.max) *(uint16_t*)o.addr = val;
  } else {
   val = eeprom_read_byte(o.addr_eep);
   if(val <= o.max) *(uint8_t*)o.addr = val;
  }
 }
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

  PORT(LED_PORT) &= ~LED;
  DDR(LED_PORT) = LED;
  PORT(BUZZ_PORT) &= ~BUZZ;
  DDR(BUZZ_PORT) |= BUZZ;
  
  TCCR0A = _BV(WGM01)|_BV(WGM00);
  OCR0A = 249; /* TOV will occur every 1ms for 16Mhz clock */
  TCCR0B = _BV(WGM02)|_BV(CS01)|_BV(CS00); /* /64 */
  TIMSK0 = _BV(TOIE0);

  PCMSK1 = _BV(PCINT11); /* SRQ */
  PCICR = _BV(PCIE1);

  sei();
  
  command = 13; /* force line edit restart */
  set_defaults(0);
  load_settings();

  if(gpib_hp3478_addr == 31) command = 'P';

  uart_init(uart_baud);
  fdevopen(uart_putchar, NULL);
  
  PORT(SRQ_PORT) |= SRQ;
  gpib_talk();

  ext_state = !hp3478_ext_enable;
#if 0
  beep(buzz_period, buzz_duty);
  for(timeout = 0; timeout < 5000; timeout++) _delay_ms(1);
  beep_off();
  timeout = 0; 
#endif
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
