# Copyright (C) 2024 Toshimitsu Kimura <lovesyao@gmail.com>
# SPDX-License-Identifier: BSD-3-Clause

ifeq ($(PREFIX),)
	PREFIX := /usr/local
endif

all: gsclock_arduino.hex

gsclock_arduino.hex: gsclock_arduino.elf
	avr-objcopy -O ihex $< $@

gsclock_arduino.elf: gsclock_arduino.ino avr-gcc-arduino.specs
	avr-gcc -specs=./avr-gcc-arduino.specs -Os -Wall -mmcu=atmega328p -ffunction-sections -fdata-sections -DF_CPU=16000000L -DARDUINO=10807 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR -I/usr/share/arduino/hardware/arduino/avr/cores/arduino -I/usr/share/arduino/hardware/arduino/avr/variants/standard -I/usr/share/arduino/hardware/arduino/avr/libraries/EEPROM/src/ /usr/share/arduino/hardware/arduino/avr/cores/arduino/*.cpp  /usr/share/arduino/hardware/arduino/avr/cores/arduino/*.c -I./Arduino-IRremote/src/ -x c++ $< -o gsclock_arduino.elf
#	avr-gcc -specs=./avr-gcc-arduino.specs -Os -g -ggdb -Wall -mmcu=atmega328p -ffunction-sections -fdata-sections -DF_CPU=16000000L -DARDUINO=10807 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR -I/usr/share/arduino/hardware/arduino/avr/cores/arduino -I/usr/share/arduino/hardware/arduino/avr/variants/standard -I/usr/share/arduino/hardware/arduino/avr/libraries/EEPROM/src/ /usr/share/arduino/hardware/arduino/avr/cores/arduino/*.cpp  /usr/share/arduino/hardware/arduino/avr/cores/arduino/*.c -I./Arduino-IRremote/src/ -x c++ $< -o gsclock_arduino.elf # for avr-objdump -S

clean:
	rm -f gsclock_arduino.hex gsclock_arduino.elf

install: gsclock_arduino.hex
	install -D -m 755 -t $(DESTDIR)$(PREFIX)/share/gsclock/ gsclock
	install -m 644 -t $(DESTDIR)$(PREFIX)/share/gsclock/ gsclock.html gsclock_arduino.elf avr-gcc-arduino.specs gsclock_arduino_uno_sketch.png gsclock_arduino.ino gsclock_arduino.hex
	install -D -m 644 -t $(DESTDIR)$(PREFIX)/share/gsclock/styles  styles/*
	mkdir -p $(DESTDIR)$(PREFIX)/bin/
	ln -sr $(DESTDIR)$(PREFIX)/share/gsclock/gsclock $(DESTDIR)$(PREFIX)/bin/gsclock

uninstall:
	rm -rf $(DESTDIR)$(PREFIX)/share/gsclock/
	rm -f $(DESTDIR)$(PREFIX)/bin/gsclock

deb:
	debuild

# build_py? build_ext? bdist_wheel?
