# Target name of final executable
TARGET = bare_metal

# DEBUG
DEBUG = 1

# Build Directory
BUILD_DIR = build

# Tools to use
CROSS_COMPILE = arm-none-eabi-
CC = $(CROSS_COMPILE)gcc
LD = $(CROSS_COMPILE)gcc
CP = $(CROSS_COMPILE)objcopy


# GCC FLAGS
CPU = -mcpu=cortex-m4
FPU = -mfpu=fpv4-sp-d16
FLOAT-ABI = -mfloat-abi=hard
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# Additional DEBUG Flag
OPT = -O3
DEBUG_FLAG = -Wall -Wno-unused -Wextra -pedantic

# -H print the recursive searh path
# -M prints which headerfiles was included
ifeq ($(DEBUG), 1)
OPT = -O0
DEBUG_FLAG += -H -g3 -gdwarf-2
endif

# Compiler Flags
#CFLAGS += -ffreestanding
CFLAGS += $(MCU) $(DEBUG_FLAG) $(OPT)


# Linker Script
LSCRIPT=mem.ld

# Libs for Linker
# gonna use -nostdlib and pick and choose desired libs
# -lc has malloc but need to implement _sbrk
LIBS =-lc
# Linker Flags
LDFLAGS = -T$(LSCRIPT)  $(MCU) -nostdlib $(LIBS)

# Objs
#OBJ = startup.o

all: $(TARGET).bin

$(TARGET).bin: $(TARGET).elf
	$(CP) -O binary $< $@

$(TARGET).elf: startup.o
	$(LD) startup.o $(LDFLAGS) -o bare_metal.elf

startup.o: startup.c
	$(CC) $(CFLAGS) -c startup.c

.PHONY: clean nuke

make nuke:
	-rm -rf *.o *.elf *.bin *.hex
