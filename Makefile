all: gsclock_arduino.hex

gsclock_arduino.hex: gsclock_arduino.elf
	avr-objcopy -O ihex $< $@

gsclock_arduino.elf: gsclock_arduino.ino avr-gcc-arduino.specs
	avr-gcc -specs=./avr-gcc-arduino.specs -Os -Wall -mmcu=atmega328p -ffunction-sections -fdata-sections -DF_CPU=16000000L -DARDUINO=10807 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR -I/usr/share/arduino/hardware/arduino/avr/cores/arduino -I/usr/share/arduino/hardware/arduino/avr/variants/standard  /usr/share/arduino/hardware/arduino/avr/cores/arduino/*.cpp  /usr/share/arduino/hardware/arduino/avr/cores/arduino/*.c -I./Arduino-IRremote/src/ -x c++ $< -o gsclock_arduino.elf

# build_py? build_ext? bdist_wheel?
