
CC=avr-gcc
MCU=atmega328p
OFLAG=-O2

NAME = hp3478-ext

LDFLAGS = -Wl,-Map,$(NAME).map
CFLAGS=  $(OFLAG) -g -Wall -mmcu=$(MCU) -ffreestanding -Wa,-ahlms=$(<:.c=.lst)

.SUFFIXES: .s .bin .out .hex

.c.s:
	$(CC) $(CFLAGS) -S $<

.S.o:
	$(CC) $(ASFLAGS) -c $<

.o.out:
	$(CC) $(CFLAGS) -o $@ $<

.out.bin:
	avr-objcopy -O binary $< $@

.out.hex:
	avr-objcopy -O ihex $< $@

all:	$(NAME).hex

OBJS = $(NAME).o uart.o

$(NAME).out: $(OBJS)
	$(CC) -o $(NAME).out $(CFLAGS) $(LDFLAGS) $(OBJS) $(LDLIBS)
	avr-size $(NAME).out

clean:
	rm -f *.out *.bin *.hex *.s *.o


