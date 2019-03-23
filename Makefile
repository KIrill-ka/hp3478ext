
CC=avr-gcc
MCU=atmega328p
OFLAG=-O2

NAME = hp3478-ext

LDFLAGS = -Wl,-Map,$(NAME).map
CFLAGS=  $(OFLAG) -g -Wall -mmcu=$(MCU) -ffreestanding -Wa,-ahlms=$(<:.c=.lst)

.SUFFIXES: .s .bin .out .hex .eep

.c.s:
	$(CC) $(CFLAGS) -S $<

.S.o:
	$(CC) $(ASFLAGS) -c $<

.o.out:
	$(CC) $(CFLAGS) -o $@ $<

.out.bin:
	avr-objcopy -O binary $< $@

.out.hex:
	avr-objcopy -R .eeprom -R .fuse -R .lock -O ihex $< $@

#.out.eep:
#avr-objcopy -j .eeprom --set-section-flags=.eeprom="alloc,load" \
#--change-section-lma .eeprom=0 -O ihex $< $@
# --no-change-warnings

all:	$(NAME).hex $(NAME).eep $(NAME)-dc-buzzer.eep

$(NAME)-dc-buzzer.eep: eepmap.h
	./eeprom_set_var.tcl -d DEF2 $@

$(NAME).eep: eepmap.h
	./eeprom_set_var.tcl -d DEF1 $@

OBJS = $(NAME).o uart.o

$(NAME).out: $(OBJS)
	$(CC) -o $(NAME).out $(CFLAGS) $(LDFLAGS) $(OBJS) $(LDLIBS)
	avr-size -A $(NAME).out

clean:
	rm -f *.out *.bin *.hex *.s *.o


