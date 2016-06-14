include ../py/mkenv.mk

CROSS = 1
DEBUG = 1

# qstr definitions (must come before including py.mk)
QSTR_DEFS = qstrdefsport.h

# qstr definitions (must come before including py.mk)
QSTR_DEFS = $(BUILD)/pins_qstr.h

# include py core make definitions
include ../py/py.mk

ifeq ($(CROSS), 1)
CROSS_COMPILE = arm-none-eabi-
endif

INC += -I.
INC += -I..
INC += -I../lib/mp-readline
#INC += -I../stmhal
INC += -Iutil
INC += -Ihal
INC += -Imods
INC += -Imisc
INC += -I$(BUILD)
INC += -I$(HEADER_BUILD)

ifeq ($(CROSS), 1)
DFU = ../tools/dfu.py
PYDFU = ../tools/pydfu.py
CFLAGS_CORTEX_M4 = -mthumb -mtune=cortex-m4 -mabi=aapcs-linux -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -fsingle-precision-constant -Wdouble-promotion
CFLAGS = $(INC) -Wall -ansi -std=gnu99 -nostdlib $(CFLAGS_CORTEX_M4) $(COPT)  #-Werror 
else
CFLAGS = -m32 $(INC) -Wall -Werror -ansi -std=gnu99 $(COPT)
endif

#Debugging/Optimization
ifeq ($(DEBUG), 1)
CFLAGS += -O0 -ggdb
else
CFLAGS += -Os -DNDEBUG
endif

ifeq ($(CROSS), 1)
LDFLAGS = -nostdlib -T tm4c123gh6pm.lds -Map=$@.map --cref
else
LD = gcc
LDFLAGS = -m32 -Wl,-Map=$@.map,--cref
endif
LIBS =

SRC_C = \
	main.c \
	uart_core.c \
	uart_extra.c \
	gccollect.c \
	lib/utils/printf.c \
	lib/utils/pyexec.c \
	lib/libc/string0.c \
	lib/mp-readline/readline.c \
	$(BUILD)/pins.c \
	$(BUILD)/_frozen_mpy.c \

SRC_S = \
	gchelper.s \


APP_MODS_SRC_C = $(addprefix mods/,\
	modwipy.c \
	modmachine.c \
	pybpin.c \
	)

#	antenna.c \
#	FreeRTOSHooks.c \
#	help.c \
#	mpirq.c \
#	mperror.c \
#	mpsystick.c \

APP_MISC_SRC_C = $(addprefix misc/,\
	mpexception.c \
	mpirq.c \
	)

APP_HAL_SRC_C = $(addprefix hal/,\
	gpio.c \
	)


# List of sources for qstr extraction
SRC_QSTR += $(APP_MODS_SRC_C) $(APP_MISC_SRC_C) #$(APP_STM_SRC_C)

OBJ = $(PY_O) $(addprefix $(BUILD)/, $(SRC_C:.c=.o) $(SRC_S:.s=.o) $(APP_MODS_SRC_C:.c=.o) $(APP_MISC_SRC_C:.c=.o) $(APP_HAL_SRC_C:.c=.o))
#OBJ += $(addprefix $(BUILD)/, $(SRC_S:.s=.o))

ifeq ($(CROSS), 1)
all: $(BUILD)/firmware.dfu $(BUILD)/micropython.out
else
all: $(BUILD)/firmware.elf
endif

$(BUILD)/_frozen_mpy.c: frozentest.mpy $(BUILD)/genhdr/qstrdefs.generated.h
	$(ECHO) "MISC freezing bytecode"
	$(Q)../tools/mpy-tool.py -f -q $(BUILD)/genhdr/qstrdefs.preprocessed.h -mlongint-impl=none $< > $@

$(BUILD)/firmware.elf: $(OBJ)
	$(ECHO) "LINK $@"
	$(Q)$(LD) $(LDFLAGS) -o $@ $^ $(LIBS)
	$(Q)$(SIZE) $@

$(BUILD)/micropython.out: $(OBJ)
	$(ECHO) "LINK $@"
	$(Q)$(LD) $(LDFLAGS) -o $@ $^ $(LIBS)
	$(Q)$(SIZE) $@

$(BUILD)/firmware.dfu: $(BUILD)/firmware.elf
	$(ECHO) "Create $@"
	$(Q)$(OBJCOPY) -O binary -j .isr_vector -j .text -j .data $^ $(BUILD)/firmware.bin
	$(Q)$(PYTHON) $(DFU) -b 0x08000000:$(BUILD)/firmware.bin $@

deploy: $(BUILD)/firmware.dfu
	$(ECHO) "Writing $< to the board"
	$(Q)$(PYTHON) $(PYDFU) -u $<

# Run emulation build on a POSIX system with suitable terminal settings
run:
	stty raw opost -echo
	build/firmware.elf
	@echo Resetting terminal...
# This sleep is useful to spot segfaults
	sleep 1
	reset

test: $(BUILD)/firmware.elf
	$(Q)/bin/echo -e "print('hello world!', list(x+1 for x in range(10)), end='eol\\\\n')\\r\\n\\x04" | $(BUILD)/firmware.elf | tail -n2 | grep "^hello world! \\[1, 2, 3, 4, 5, 6, 7, 8, 9, 10\\]eol"


include ../py/mkrules.mk
