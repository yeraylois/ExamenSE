OS := $(shell uname -s)

OPEN_TERMINAL := \
if [ "$(OS)" = "Darwin" ]; then \
    osascript -e 'tell application "Terminal" to do script "bash $(PWD)/open_screen.sh"'; \
elif [ "$(OS)" = "Linux" ]; then \
    gnome-terminal -- bash -c "$(PWD)/open_screen.sh"; \
else \
    echo "Unsupported OS"; \
    exit 1; \
fi

prepare_screen:
	echo '#!/bin/bash' > open_screen.sh
	echo 'screen /dev/tty.usb* 115200' >> open_screen.sh
	chmod +x open_screen.sh

CC = arm-none-eabi-gcc
AS = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
SIZE = arm-none-eabi-size

CFLAGS = -I./BOARD -I./drivers -I./include -I./utilities -O0 -Wall -mthumb -mcpu=cortex-m0plus -DCPU_MKL46Z128VLH4
LDFLAGS = -O0 -Wall -Wextra -mthumb -mcpu=cortex-m0plus --specs=nosys.specs -Wl,--gc-sections,-Map=output.map -T MKL46Z256xxx4_flash.ld

C_SOURCES = main.c \
            reverse1.c \
            reverse4.c \
			$(wildcard BOARD/*.c) \
			$(wildcard drivers/*.c) \
			$(wildcard include/*.c) \
			$(wildcard utilities/*.c)

ASM_SOURCES = startup_MKL46Z4.S reverse2.S
              #reverse3.s

OBJ = $(C_SOURCES:.c=.o) $(ASM_SOURCES:.S=.o)
TARGET = program.elf
OPENOCD_CFG = openocd.cfg

all: $(TARGET)

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

%.o: %.S
	$(AS) $(CFLAGS) -c -o $@ $<

$(TARGET): $(OBJ)
	$(CC) $(LDFLAGS) $(OBJ) -o $(TARGET)
	$(SIZE) $(TARGET)

flash: $(TARGET)
	openocd -f $(OPENOCD_CFG) -c "program $(TARGET) verify reset exit"

flash_t: $(TARGET) prepare_screen
	$(OPEN_TERMINAL); \
	openocd -f $(OPENOCD_CFG) -c "program $(TARGET) verify reset exit"

clean:
	rm -f $(OBJ) $(TARGET) output.map open_screen.sh
