# Target name of final executable
TARGET = bare_metal

# DEBUG
DEBUG = 1

# Directories
LSCRIPT_DIR= linker
BUILD_DIR = build
HAL_DIR = hal
MEM_MGT_DIR = mem_mgt
MU_DIR = mu
STARTUP_DIR = startup
VEC_TABLE_DIR = vector_table
SRC_FILES = $(wildcard  *.c) \
						$(wildcard $(HAL_DIR)/*.c) \
						$(wildcard $(MEM_MGT_DIR)/*.c) \
						$(wildcard $(MU_DIR)/*.c) \
						$(wildcard $(STARTUP_DIR)/*.c) \
						$(wildcard $(VEC_TABLE_DIR)/*.c)

# Strip the ./ with ./%=% to produce object files
OBJ_FILES = $(patsubst %.c,$(BUILD_DIR)/%.o,$(SRC_FILES))
BUILD_FILES = $(OBJ_FILE)
DEP_FILES = $(OBJ_FILES:.o=.d)
$(info SRC_FILES is $(SRC_FILES))
$(info OBJ_FILES is $(OBJ_FILES))
$(info BUILD_FILES is $(OBJ_FILES))
$(info DEP_FILES is $(DEP_FILES))

#Include Directories
INC_DIR = I$(HAL_DIR)/include \
					-I$(LSCRIPT_DIR) \
					-I$(MU_DIR) \
					-I$(STARTUP_DIR)

$(info INC_DIR is $(INC_DIR))

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
GCC_STANDARD = -std=c90
DEPENDECY = -MMD

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
CFLAGS += $(MCU) $(DEBUG_FLAG) $(OPT) $(GCC_STANDARD) $(DEPENDECY)\
					-$(INC_DIR) \
					-ffreestanding -ffunction-sections -nostdlib \
					-fdata-sections


# Linker Script
LSCRIPT=$(LSCRIPT_DIR)/mem.ld
$(info Linker is $(LSCRIPT))

# Libs for Linker
# gonna use -nostdlib and pick and choose desired libs
# -lc has malloc but need to implement _sbrk
LIBS = -lc
# Linker Flags
LDFLAGS = -T$(LSCRIPT)  $(MCU) -nostdlib $(LIBS) -Wl,-Map=map.map -Wl,--cref -Wl,--gc-sections

# Objs
#OBJ = startup.o

all: $(TARGET).bin

$(TARGET).bin: $(TARGET).elf | $(BUILD_DIR)
	$(CP) -O binary $< $@

$(TARGET).elf: $(OBJ_FILES) | $(BUILD_DIR)
	$(LD)  $(LDFLAGS) -o $@ $^

$(BUILD_FILES)/%.o: $(SRC_FILES) | $(BUILD_DIR)
	$(CC) $(CFLAGS)  -c $< -o $@

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)
#startup.o: startup.c
	#$(CC) $(CFLAGS) -c startup.c

#vector_table.o: vector_table.c
	#$(CC) $(CFLAGS) -c vector_table.c

#dynamic_memory.o: dynamic_memory.c
#	$(CC) $(CFLAGS) -c dynamic_memory.c

#main.o: main.c
#	$(CC) $(CFLAGS) -c main.c


.PHONY: clean nuke list
-include $(DEP_FILES)

list:
	@$(MAKE) -pRrq -f $(lastword $(MAKEFILE_LIST)) : 2>/dev/null | awk -v RS= -F: '/^# File/,/^# Finished Make data base/ {if ($$1 !~ "^[#.]") {print $$1}}' | sort | egrep -v -e '^[^[:alnum:]]' -e '^$@$$'

make nuke:
	-rm -rf *.o *.d *.elf *.bin *.hex *.map $(BUILD_DIR)
