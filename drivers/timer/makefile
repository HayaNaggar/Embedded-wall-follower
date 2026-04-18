# -----------------------------------------------------------------------
# Makefile — Timer driver + LED test for ATmega328P (Arduino Uno)
#
# Usage:
#   make          -> compile everything to .hex
#   make flash    -> compile and upload to Arduino Uno
#   make clean    -> remove build artifacts
#
# Change PORT to your actual COM port:
#   Windows:  COM3, COM4, etc.
#   Linux:    /dev/ttyUSB0 or /dev/ttyACM0
#   macOS:    /dev/cu.usbmodem...
# -----------------------------------------------------------------------

MCU     = atmega328p
F_CPU   = 16000000UL
BAUD    = 115200
PORT    = COM10          # <-- change this to your port

CC      = avr-gcc
OBJCOPY = avr-objcopy
AVRDUDE = avrdude

CFLAGS  = -mmcu=$(MCU) -DF_CPU=$(F_CPU) -Os -Wall -std=c99

TARGET  = test_timer_led
SRCS    = test_timer_led.c timer.c

# -----------------------------------------------------------------------

all: $(TARGET).hex

$(TARGET).elf: $(SRCS)
	$(CC) $(CFLAGS) -o $@ $^

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@

flash: $(TARGET).hex
	$(AVRDUDE) -c arduino -p $(MCU) -P $(PORT) -b $(BAUD) -U flash:w:$<

clean:
	del $(TARGET).elf $(TARGET).hex   # Windows
	# rm -f $(TARGET).elf $(TARGET).hex  # Linux/macOS: uncomment this line