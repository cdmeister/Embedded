PREFIX=arm-none-eabi-
CC=$(PREFIX)gcc
LD=$(PREFIX)ld
OBJCOPY=$(PREFIX)objcopy

CPU = -mcpu=cortex-m4
FPU = -mfpu=fpv4-sp-d16
FLOAT_ABI = -mfloat-abi=hard

LSCRIPT=mem.ld

DEBUG = -H -g -O0 -gdwarf-4 -ggdb3


CFLAGS=$(CPU) -mthumb $(FPU) $(FLOAT_ABI) $(DEBUG) -ffreestanding -nostdlib
LDFLAGS = -T $(LSCRIPT) -gc-sections -Map=image.map -nostdlib

#all: image.bin

image.bin: image.elf
	$(OBJCOPY) -O binary $^ $@

image.elf: startup.o mem.ld
	$(LD) $(LDFLAGS) startup.o -o $@

startup.o: startup.c

.PHONY : clean

clean:
	rm -f image.bin image.elf *.o image.map
