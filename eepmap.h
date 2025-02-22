#pragma once

#define EEP_ADDR_BEEP_DUTY        0
#define EEP_ADDR_BEEP_PERIOD      1 /* 2 */

#define EEP_ADDR_UART_BAUD        3
#define EEP_ADDR_UART_ECHO        9

#define EEP_ADDR_GPIB_END_SEQ_TX  4
#define EEP_ADDR_GPIB_END_SEQ_RX  5
#define EEP_ADDR_GPIB_HP3478_ADDR 7
#define EEP_ADDR_GPIB_MY_ADDR     8

#define EEP_ADDR_HP3478_EXT_EN   10

#define EEP_ADDR_TEMP_2W_OFF     16 /* 2 */
#define EEP_ADDR_TEMP_R0_OFF     18 /* 2 */
#define EEP_ADDR_CONT_RANGE      20
#define EEP_ADDR_CONT_THRESHOLD  24
#define EEP_ADDR_CONT_LATCH      28
#define EEP_ADDR_CONT_BEEP_T1    32
#define EEP_ADDR_CONT_BEEP_T2    36
#define EEP_ADDR_CONT_BEEP_D1    40
#define EEP_ADDR_CONT_BEEP_D2    44
#define EEP_ADDR_CONT_BEEP_P1    48 /* 2 */
#define EEP_ADDR_CONT_BEEP_P2    52 /* 2 */
#define EEP_ADDR_DBM_REF         56 /* 2 */

#define EEP_ADDR_MODE            60 /* 2 */
#define EEP_ADDR_EXT_MODE        62

#define EEP_ADDR_ERR_DISP        70

#define EEP_SIZE_BEEP_PERIOD      2
#define EEP_SIZE_CONT_BEEP_P1     2
#define EEP_SIZE_CONT_BEEP_P2     2
#define EEP_SIZE_CONT_THRESHOLD   2
#define EEP_SIZE_CONT_BEEP_T1     2
#define EEP_SIZE_CONT_BEEP_T2     2
#define EEP_SIZE_MODE             2

/* set 0, used for uninitalized eeprom */
#define EEP_DEF0_BEEP_DUTY        15
#define EEP_DEF0_BEEP_PERIOD      10000

#define EEP_DEF0_UART_BAUD        0 /* UART_115200 */
#define EEP_DEF0_UART_ECHO        1

#define EEP_DEF0_GPIB_END_SEQ_TX  4 /* GPIB_END_EOI */
#define EEP_DEF0_GPIB_END_SEQ_RX  4
#define EEP_DEF0_GPIB_HP3478_ADDR 23
#define EEP_DEF0_GPIB_MY_ADDR     21

#define EEP_DEF0_HP3478_EXT_EN   0

#define EEP_DEF0_TEMP_2W_OFF     0
#define EEP_DEF0_TEMP_R0_OFF     0
#define EEP_DEF0_CONT_RANGE      1  /* 300 Ohm */
#define EEP_DEF0_CONT_THRESHOLD  1000 /* 100 ohm in 300 Ohm range */
#define EEP_DEF0_CONT_LATCH      0  /* no latch */
#define EEP_DEF0_CONT_BEEP_T1    1000
#define EEP_DEF0_CONT_BEEP_T2    1000
#define EEP_DEF0_CONT_BEEP_D1    15
#define EEP_DEF0_CONT_BEEP_D2    15
#define EEP_DEF0_CONT_BEEP_P1    10000
#define EEP_DEF0_CONT_BEEP_P2    10000
#define EEP_DEF0_DBM_REF         50
#define EEP_DEF0_MODE            0
#define EEP_DEF0_EXT_MODE        0
#define EEP_DEF0_ERR_DISP        1

/* set 1, used for the default .eep (differences to the set 0) */
#define EEP_DEF1_HP3478_EXT_EN   1

/* set 2, DC buzzer configuration */
#define EEP_DEF2_HP3478_EXT_EN   1
#define EEP_DEF2_CONT_BEEP_P1    0
#define EEP_DEF2_CONT_BEEP_P2    0
#define EEP_DEF2_BEEP_PERIOD     0

#define EEP_PRESET_SIZE  128
