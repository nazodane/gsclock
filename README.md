The Good-shaped Clock (gsclock; ja: 良い感じの時計) is a virtual analog clock with Arduino's sensor data for Linux desktop!

Requirements
============
* Ubuntu 22.04
* [ELEGOO's UNO R3 Most Complete Starter Kit](https://www.elegoo.com/en-jp/products/elegoo-uno-most-complete-starter-kit)（ja: [最終版スタータキット](https://www.amazon.co.jp/dp/B06Y56JV64)）
** Arduino UNO R3
** Breadboard
** Photoresistor
** DHT11
** 10kΩ register
** 1kΩ register x2
** Red LED
** Button
** many jumper wires

Installation
============
1. Setup your Arduino Uno R3 like this (XXX: not safety-verified):
![Arduino Setup Image](gsclock_arduino_uno_sketch.png)

2. Download and install the Arduino IDE from [the official site](https://www.arduino.cc/en/software).

3. Download the DHT-Sensors-Non-Blocking-1.0.4.zip file from [toannv17/DHT-Sensors-Non-Blocking (DHT_Async) releases](https://github.com/toannv17/DHT-Sensors-Non-Blocking/releases).

4. Open the '''Arduino IDE'''.

5. Select ''Sketch'' -> ''Include Library'' -> ''Add .zip Library'' in Arduino IDE's menu, and then select DHT-Sensors-Non-Blocking-1.0.4.zip.

6. Open ''gsclock_arduino/gsclock_arduino.ino'' source code in Arduino IDE, and then upload it.

7.  Install chromium-chromedriver, the browser-based application environment.
```bash
sudo apt install chromium-chromedriver
```

8. Install the main program of gsclock!
```bash
pip install git+https://github.com/nazodane/gsclock.git
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
I just realized the analog clock is important which was forgotten. I also thought to need the remembering of the current year more often because of preventing my health problem.

I researched the real analog clock with displaying the year but I couldn't find it. I also researched virtual clocks on Linux but they are ... well... not good shaped at all.

Finally, I researched web analog clocks, and I was certain that the web analog clock is the right way so I recreated it to meet all my demands.
