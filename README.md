The Good-shaped Clock (gsclock; ja: 良い感じの時計) is a virtual analog clock with Arduino's sensor data for Linux desktop!

Requirements
============
* Ubuntu 22.04
* [ELEGOO's UNO R3 Most Complete Starter Kit](https://www.elegoo.com/en-jp/products/elegoo-uno-most-complete-starter-kit)（ja: [最終版スタータキット](https://www.amazon.co.jp/dp/B06Y56JV64)）
  * Arduino UNO R3
  * Breadboard
  * Photoresistor
  * DHT11
  * 10kΩ register
  * 1kΩ register x2
  * Red LED
  * Button
  * many jumper wires

Installation
============
1. Install the following packages via apt:
* psmisc, the small utilities for processes
* chromium-chromedriver, the browser-based application environment
* gcc-avr and avr-libc, the [cross compiler](https://en.wikipedia.org/wiki/Cross_compiler) for Arduino MPU
* arduino-core-avr, the core library for Arduino
* avrdude, the uploading utility for Arduino
```bash
sudo apt install psmisc chromium-chromedriver gcc-avr avr-libc arduino-core-avr avrdude
```

2. Setup your Arduino Uno R3 like this (XXX: not safety-verified yet):
![Arduino Setup Image](gsclock_arduino_uno_sketch.png)

3. Install the main script of gsclock and its dependencies via pip!
```bash
pip install git+https://github.com/nazodane/gsclock.git
```

4. Compile the arduino program via avr-gcc (or just open ~/.local/share/gsclock/gsclock_arduino.ino in Arduino IDE)
```bash
cd /tmp
avr-gcc -specs=$HOME/.local/share/gsclock/avr-gcc-arduino.specs -Os -Wall -mmcu=atmega328p -ffunction-sections -fdata-sections -DF_CPU=16000000L -DARDUINO=10807 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR -I/usr/share/arduino/hardware/arduino/avr/cores/arduino -I/usr/share/arduino/hardware/arduino/avr/variants/standard  /usr/share/arduino/hardware/arduino/avr/cores/arduino/*.cpp  /usr/share/arduino/hardware/arduino/avr/cores/arduino/*.c -x c++ ~/.local/share/gsclock/gsclock_arduino.ino -o gsclock_arduino.elf
```

5. Upload the arduino program to the your Arduino Uno R3 via avrdude (or just upload via Arduino IDE)
```bash
avr-objcopy -O ihex gsclock_arduino.elf gsclock_arduino.hex
avrdude -c arduino -p atmega328p -b 115200 -P /dev/ttyACM0 -U flash:w:gsclock_arduino.hex
```


Usage
=====
```bash
~/.local/bin/gsclock
```

License
=======
The Good-shaped Clock is mainly under BSD-3-Clause License. The arduino part is under Apache-2.0 license. The images are under CC-0 license and the font is under SIL Open Font License.

For more details, see [LICENSE](LICENSE) file.

Screenshot
==========
![Good-shaped Clock Screenshot Image](gslock.png)

Donations
=========
My Amazon wishlist: https://www.amazon.co.jp/hz/wishlist/ls/CCPOV7C6JTD2

Motivations
===========
I've just come to realize the importance of using an analog clock, which I had completely overlooked. Additionally, I've realized the necessity of keeping track of the current year more frequently as a means to prevent my health issues.

I attempted to find a physical analog clock that also displayed the year but my search was unsuccessful. I then looked into virtual clocks for Linux, but unfortunately, they didn't meet my requirements at all.

Finally, I turned to web analog clocks and found that they were the perfect solution. I recreated one to suit all of my needs.
