# Makefile for Arduino Nano / ATmega328P build and flash

MCU ?= atmega328p
F_CPU ?= 16000000UL
CC ?= avr-gcc
OBJCOPY ?= avr-objcopy
AVRDUDE ?= avrdude
PROGRAMMER ?= arduino
PORT ?= COM3
BAUD ?= 115200
CFLAGS ?= -mmcu=$(MCU) -DF_CPU=$(F_CPU) -Os -std=c11 -Wall
INCLUDES ?= -Iapp -Idrivers/gpio -Idrivers/sensors/IR -Idrivers/uart -Idrivers/timer -Idrivers/pwm -Imodules/motor
SRC ?= app/main.c drivers/gpio/gpio.c drivers/sensors/IR/IR_sensor.c drivers/uart/uart.c drivers/timer/timer.c drivers/pwm/pwm.c modules/motor/motor.c
TARGET_DIR ?= build
TARGET_BASE ?= robot
ELF ?= $(TARGET_DIR)/$(TARGET_BASE).elf
HEX ?= $(TARGET_DIR)/$(TARGET_BASE).hex

.PHONY: all build clean flash

all: $(HEX)

build: $(ELF)

$(TARGET_DIR):
	mkdir "$(TARGET_DIR)" 2>nul || true

$(ELF): $(SRC) | $(TARGET_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) $(SRC) -o $@

$(HEX): $(ELF)
	$(OBJCOPY) -O ihex -R .eeprom $< $@

flash: $(HEX)
	$(AVRDUDE) -p $(MCU) -c $(PROGRAMMER) -P $(PORT) -b $(BAUD) -U flash:w:$(HEX):i

clean:
	rm -f $(ELF) $(HEX)
