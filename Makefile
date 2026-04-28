# =========================
# AVR ATmega328P Makefile
# =========================

MCU = atmega328p
F_CPU = 16000000UL

CC = avr-gcc
OBJCOPY = avr-objcopy
AVRDUDE = avrdude

PROGRAMMER = arduino
PORT = /dev/ttyUSB0
BAUD = 57600

CFLAGS = -mmcu=$(MCU) -DF_CPU=$(F_CPU) -Os -std=c11 -Wall

INCLUDES = -Iapp \
           -Idrivers/gpio \
           -Idrivers/sensors/IR \
           -Idrivers/uart \
           -Idrivers/timer \
           -Idrivers/pwm \
           -Imodules/motor

SRC = app/main.c \
      drivers/gpio/gpio.c \
      drivers/sensors/IR/IR_sensor.c \
      drivers/uart/uart.c \
      drivers/timer/timer.c \
      drivers/pwm/pwm.c \
      modules/motor/motor.c

TARGET_DIR = build
TARGET_BASE = robot

ELF = $(TARGET_DIR)/$(TARGET_BASE).elf
HEX = $(TARGET_DIR)/$(TARGET_BASE).hex

.PHONY: all build clean flash

# Default target
all: build

# Build everything
build: $(ELF) $(HEX)

# =========================
# FIXED ELF RULE (IMPORTANT)
# =========================
$(ELF): $(SRC)
	mkdir -p $(TARGET_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) $(SRC) -o $(ELF)

# HEX generation
$(HEX): $(ELF)
	$(OBJCOPY) -O ihex -R .eeprom $(ELF) $(HEX)

# Flash to Arduino
flash: $(HEX)
	$(AVRDUDE) -p $(MCU) -c $(PROGRAMMER) -P $(PORT) -b $(BAUD) -U flash:w:$(HEX):i

# Clean build
clean:
	rm -rf $(TARGET_DIR)