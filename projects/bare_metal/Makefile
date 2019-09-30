PREFIX=arm-none-eabi-
CC=$(PREFIX)gcc
LD=$(PREFIX)ld
OBJCOPY=$(PREFIX)objcopy

CPU = -mcpu=cortex-m4
FPU = -mfpu=fpv4-sp-d16
FLOAT_ABI = -mfloat-abi=hard

DEBUG = -g -ggdb -Wall -Wno-main

CFLAGS=$(CPU) -mthumb $(FPU) $(FLOAT_ABI) $(DEBUG)
LDFLAGS:=-T mem.ld -gc-sections -nostdlib -Map=image.map

#all: image.bin

image.bin: image.elf
	$(OBJCOPY) -O binary $^ $@

image.elf: startup.o mem.ld
	$(LD) $(LDFLAGS) startup.o -o $@

startup.o: startup.c

.PHONY : clean

clean:
	rm -f image.bin image.elf *.o image.map
