/*
Copyright (C) 2024 Toshimitsu Kimura <lovesyao@gmail.com>
SPDX-License-Identifier: Apache-2.0
*/

#include "DHT-Sensors-Non-Blocking/DHT_Async.cpp" // for temperature and humidity

#include "Tone/Tone.h" // for tone frequency defination

#define IR_USE_AVR_TIMER1 // timer1 for ir recv and timer2 for tone
#define RAW_BUFFER_LENGTH 100 // just in case
#define MICROS_PER_TICK 50 // just in case
#define NO_DECODER // just resend the raw data so the decoders are not needed
#include "Arduino-IRremote/src/IRremote.hpp" // or IRMP?

#include <EEPROM.h> // save the IR remote control code for home lighting. lowest on/off/up/down?

const char *IR_CODE_NAMES[] = { "full light", "night light",  "light out", "lighter", "darker"};
#define IR_CODE_LEN sizeof(IR_CODE_NAMES) / sizeof(IR_CODE_NAMES[0])

#define IR_LIGHTING_FULL 0
#define IR_LIGHTING_NIGHT 1
#define IR_LIGHTING_OUT 3
#define IR_LIGHTING_LIGHTER 4
#define IR_LIGHTING_DARKER 5

#define EEPROM_IR_START 4
// EEPROM specification of the device:
// EEPROM[  0..  4] = the EEPROM version like 'GS00'
// EEPROM[  4..104] = LIGHTING FULL LIGHT RAW IR CODE (each element is compressed by (uint8_t)(code / MICROS_PER_TICK); 50μs * 255 = 12.75ms should be sufficient.)
// EEPROM[104..204] = LIGHTING NIGHT LIGHT RAW IR CODE (ditto)
// EEPROM[204..304] = LIGHTING LIGHT OUT RAW IR CODE (ditto)
// EEPROM[304..404] = LIGHTING LIGHTER RAW IR CODE (ditto)
// EEPROM[404..504] = LIGHTING DARKER RAW IR CODE (ditto)

/*
Unique code for gsclock
*/
#define DHT_SENSOR_TYPE DHT_TYPE_11

static const int DHT_SENSOR_PIN = 7; // D7

/*
Copyright (C) 2023 Toan Nguyen
SPDX-License-Identifier: Apache-2.0
based on https://github.com/toannv17/DHT-Sensors-Non-Blocking/blob/a195a0a05c314923bdb528e61b4ef2672a022b55/examples/DHTAsyncTester/DHTAsyncTester.ino
*/
DHT_Async dht_sensor(DHT_SENSOR_PIN, DHT_SENSOR_TYPE);

static bool measure_environment(float *temperature, float *humidity) {
    static unsigned long measurement_timestamp = millis();

    if (millis() - measurement_timestamp > 2000ul) { // XXX: modified by gsclock
        if (dht_sensor.measure(temperature, humidity)) {
            measurement_timestamp = millis();
            return (true);
        }
    }

    return (false);
}

#define FALSE 0
#define TRUE 1

int powerButtonPin = 2; // D2
int powerLedPin = 4; // D4
int activeBuzzerPin = 6; //D6
int irReceiveButtonPin = 10; //D10
int irLedPin = 9; // D9
int irReceivePin = 5;  // D5
int irSendPin = 3; // D3
int pirPin = 11; // D11
int photoresistorPin = 0; // A0
int direct3V3VoltagePin = 5;  // A5
int currentButtonState = FALSE;
int currentIrButtonState = FALSE;

//bool currentWorkingState = FALSE;
bool currentWorkingState = TRUE;

bool currentIrModeState = FALSE;
size_t irWaintingCode = 0;

void print_power_change() {
  Serial.print("mesg: power ");
  Serial.println(currentWorkingState?"on":"off");
}

void print_irreceiver_change() {
  Serial.print("mesg: ir code registration ");
  Serial.print(currentIrModeState?"on; waiting ":"off");

  if (currentIrModeState) {
    Serial.print(IR_CODE_NAMES[irWaintingCode]);
    Serial.println(" code...");
  } else if (irWaintingCode == IR_CODE_LEN - 1)
    Serial.println("; the registration succeeded!");
  else
    Serial.println();
}

void setup() {
  Serial.begin(9600);
  Serial.println("mesg: GSClock version 0.0; see https://github.com/nazodane/gsclock");

  pinMode(powerButtonPin, INPUT);
  pinMode(powerLedPin, OUTPUT);
  pinMode(irReceiveButtonPin, INPUT);
  pinMode(irLedPin, OUTPUT);
  IrSender.begin(irSendPin);
  pinMode(activeBuzzerPin, OUTPUT);
  print_power_change();

  if (EEPROM.read(0) != 'G' || EEPROM.read(1) != 'S' ||
      EEPROM.read(2) != '0' || EEPROM.read(3) != '0') { // EEPROM is not initialized
      EEPROM.write(0, 'G');
      EEPROM.write(1, 'S');
      EEPROM.write(2, '0');
      EEPROM.write(3, '0');
      EEPROM.write(EEPROM_IR_START + RAW_BUFFER_LENGTH * 0, 0);
      EEPROM.write(EEPROM_IR_START + RAW_BUFFER_LENGTH * 1, 0);
      EEPROM.write(EEPROM_IR_START + RAW_BUFFER_LENGTH * 2, 0);
      EEPROM.write(EEPROM_IR_START + RAW_BUFFER_LENGTH * 3, 0);
      EEPROM.write(EEPROM_IR_START + RAW_BUFFER_LENGTH * 4, 0);
      Serial.println("mesg: EEPROM initialized");
  } else
      Serial.println("mesg: EEPROM checked");
}

unsigned int rawbuf_for_send[RAW_BUFFER_LENGTH];

bool ir_send(int ir_code) {
  if (currentIrModeState) return FALSE;
  int i = 0;
  for (; i < RAW_BUFFER_LENGTH; i++) {
    int idx = EEPROM_IR_START + ir_code * RAW_BUFFER_LENGTH + i;
    rawbuf_for_send[i] = (unsigned int)EEPROM.read(idx) * MICROS_PER_TICK;
//    Serial.print(rawbuf_for_send[i]);
//    Serial.print(", ");
    if (rawbuf_for_send[i] == 0) {
      i++;
      break;
    }
  }
//  Serial.println();
//  Serial.println(i);
  IrReceiver.begin(irReceivePin, ENABLE_LED_FEEDBACK); // XXX: for working, this is needed but why?
  IrSender.sendRaw(rawbuf_for_send, i, MICROS_PER_TICK /*kHZ*/);
  IrReceiver.stop();
  return TRUE;
}


#define saveInterrupts(flags) { flags = SREG; noInterrupts(); };
#define restoreInterrupts(flags) { SREG = flags; };

// XXX: Not tested at all
void pdm_playback_on_pin6()
{
# define PDM_PIN 6
  uint8_t pin = PDM_PIN;

  uint8_t port = digitalPinToPort(pin);
  if (port == NOT_A_PIN) return;

  uint8_t timer = digitalPinToTimer(pin);
  if (timer != NOT_ON_TIMER) // when pin has PWM timer
    digitalWrite(PDM_PIN, LOW); // disable PWM timer

//  uint8_t bit = digitalPinToBitMask(pin);

  volatile uint8_t *out;
  out = portOutputRegister(port);

  uint8_t flags;

  // PORTD (D0 - D7) / PORTB (D8-D13)
  // https://docs.arduino.cc/retired/hacking/software/PortManipulation/
  if (out != &PORTD) return;

  uint8_t temp, zero = 0, cnt_low = 0, cnt_mid = 0, cnt_high = 0;
  uint8_t step = 1; // the duration time is 1s / (16MHz / (40clk * ((2**8)**3)) / step) = 41.94304s / step

/*
  // portd debug code
  Serial.print("mesg: before: ");
  Serial.println(*out, BIN);
  temp = *out;
  for (long int tt = 0; tt < 1000; tt++) {
    for (long int t = 0; t < 100; t++)
      asm volatile (
                    "	set\n" // 1clk
                    "	bld %[temp], 6\n" // 1clk
                    "	out %[portd], %[temp]\n" // 1clk
                    "	nop\n	nop\n"
                    "	nop\n	nop\n	nop\n	nop\n	nop\n"
                    "	nop\n	nop\n	nop\n	nop\n	nop\n"
                    "	nop\n	nop\n	nop\n	nop\n	nop\n"
                    "	nop\n	nop\n	nop\n	nop\n	nop\n"
                    "	nop\n	nop\n	nop\n	nop\n	nop\n"
                    "	nop\n	nop\n	nop\n	nop\n	nop\n"
                    "	nop\n	nop\n	nop\n	nop\n	nop\n"
               : [temp] "+r" (temp)
               : [portd] "M" (_SFR_IO_ADDR(PORTD)));
    for (long int t = 0; t < 100; t++)
      asm volatile (
                    "	clt\n"
                    "	bld %[temp], 6\n"
                    "	out %[portd], %[temp]\n"
                    "	nop\n	nop\n"
                    "	nop\n	nop\n	nop\n	nop\n	nop\n"
                    "	nop\n	nop\n	nop\n	nop\n	nop\n"
                    "	nop\n	nop\n	nop\n	nop\n	nop\n"
                    "	nop\n	nop\n	nop\n	nop\n	nop\n"
                    "	nop\n	nop\n	nop\n	nop\n	nop\n"
                    "	nop\n	nop\n	nop\n	nop\n	nop\n"
                    "	nop\n	nop\n	nop\n	nop\n	nop\n"
               : [temp] "+r" (temp)
               : [portd] "M" (_SFR_IO_ADDR(PORTD)));
  }
  Serial.print("mesg: after: ");
  Serial.println(*out, BIN);
  return;
*/

  Serial.println("mesg: PDM playback ready!");

/*
  // udr0 debug code - also see the client side
  Serial.flush();
  while(Serial.available()) Serial.read();
  for (long int t = 0; t < 100; t++) {
    for (long int t = 0; t < 100; t++) {
      asm volatile (
                          "lds %[temp], %[udr]\n" // 2clk
                          "sts %[udr], %[temp]\n" // 2clk
                 : [temp] "+r" (temp)
                 : [udr] "M" (_SFR_MEM_ADDR(UDR0)));
      __builtin_avr_delay_cycles(16000000 / 9600 - 4);
    }
    asm volatile (
                        "ldi %[temp], '\n'\n" // 1clk
                        "sts %[udr], %[temp]\n" // 2clk
               : [temp] "+r" (temp)
               : [udr] "M" (_SFR_MEM_ADDR(UDR0)));
    __builtin_avr_delay_cycles(16000000 / 9600 - 3);
  }

  return;
*/


  //https://forum.arduino.cc/t/change-baud-rate-at-runtime/368191/2
  Serial.flush();
  Serial.begin(400000);
  while(Serial.available()) Serial.read();

  saveInterrupts(flags); // save interrupt state and interrupt off; no timer, no PWM, no external interrupts are intended.
  // https://arduino.stackexchange.com/questions/61567/what-functions-are-disabled-with-nointerrupts


  UCSR0B &= ~RXCIE0;
  // http://www.rjhcoding.com/avrc-uart.php
  // https://garretlab.web.fc2.com/arduino/inside/hardware/arduino/avr/cores/arduino/HardwareSerial.cpp/begin.html

  /*
  UBRR0L = 2; // (16000000/(16*400,000)) - 1 = 1.5 or (16000000 / 8 / 400,000 - 1) / 2 = 2 ???
  UBRR0H = 0;
  */

  uint8_t out_new = *out;

  // DSD is 2.8224MHz but Arduino Uno R3 is 16MHz so 16MHz/5clk = 3.2Mhz is the target.
  // The serial port baud rate is 400,000Hz but it should be ok: https://forum.arduino.cc/t/set-arduino-serial-baud-rate-above-115200-230400-256000-307200-614400/96235/9

  // *** How to create the non-standard 3.2Mhz pdm sound file ***
  // based on https://archimago.blogspot.com/2021/10/measurements-look-at-dsd-and-using-sox.html
  // $ git clone https://github.com/mansr/sox/tree/master
  // $ cd sox
  // $ autoreconf -ivf && ./configure && make -j8
  // $ sudo make install
  // $ LD_LIBRARY_PATH=/usr/local/lib sox pdm_test.wav pdm_test.dsf rate -v 32000000 sdm -f sdm-8

  // Note: longest 16-bit timer (timer1) interrupt is 1s * (16,000,000 / ((2**16) * 1024)) = 238 ms so not enough.

  // avr asm sim: https://jonopriestley.github.io/avrsim/
  // avr ops: https://www.cs.shinshu-u.ac.jp/~haeiwa/m-com/instruction.html

#    define out_from_bit(x, pin) "	bst %[temp], " #x "\n" /* 1clk */\
                        "	bld %[out_new], " #pin "\n" /* 1clk */\
                        "	out %[portd], %[out_new]\n" /*1clk*/\


     // read serial port value from UDR0 without intterupts
     asm volatile (
                   "cmd_wait:" // wait 'S' command
                   "	lds %[temp], %[udr]\n" // 2clk
                   "	cpi %[temp], 0x53\n" // 1clk
                   "	brne cmd_wait\n" // 1clk on false
                   "	nop\n"
                   "	nop\n	nop\n	nop\n	nop\n	nop\n"
                   "	nop\n	nop\n	nop\n	nop\n	nop\n"
                   "	nop\n	nop\n	nop\n	nop\n	nop\n"
                   "	nop\n	nop\n	nop\n	nop\n	nop\n"
                   "	nop\n	nop\n	nop\n	nop\n	nop\n"
                   "	nop\n	nop\n	nop\n	nop\n	nop\n"
                   "	nop\n	nop\n	nop\n	nop\n	nop\n"
                   "	lds %[temp], %[udr]\n" /* 2clk; note: "in" op is not usable here because UDR0 is not an I/O register.*/
                   out_from_bit(0, 6/*PDM_PIN*/)
                   "	nop\n"
                   "	nop\n"
                   "pdm_loop:"
                   out_from_bit(1, 6/*PDM_PIN*/)
                   "	add %[cnt_low], %[step]\n"
                   "	adc %[cnt_mid], %[zero]\n"
                   out_from_bit(2, 6/*PDM_PIN*/)
                   "	adc %[cnt_high], %[zero]\n"
                   "	nop\n"
                   out_from_bit(3, 6/*PDM_PIN*/)
                   "	nop\n"
                   "	nop\n"
                   out_from_bit(4, 6/*PDM_PIN*/)
                   "	nop\n"
                   "	nop\n"
                   out_from_bit(5, 6/*PDM_PIN*/)
                   "	nop\n"
                   "	nop\n"
                   out_from_bit(6, 6/*PDM_PIN*/)
                   "	nop\n"
                   "	nop\n"
                   out_from_bit(7, 6/*PDM_PIN*/)
                   "	lds %[temp], %[udr]\n" /* 2clk */
                   out_from_bit(0, 6/*PDM_PIN*/)
                   "	brcc pdm_loop\n"  /* 2clk on true*/
                   : [temp] "+r" (temp), [out_new] "+r" (out_new), 
                     [cnt_low] "+r" (cnt_low), [cnt_mid] "+r" (cnt_mid), [cnt_high] "+r" (cnt_high), [zero] "+r" (zero), [step] "+r" (step)
                   : [udr] "M" (_SFR_MEM_ADDR(UDR0)), [portd] "M" (_SFR_IO_ADDR(PORTD))
    );

  // lpf: https://elvistkf.wordpress.com/2016/04/19/arduino-implementation-of-filters/

  // __builtin_avr_delay_cycles

/*
  UBRR0L = 0x67; // XXX: we shold recover previous rate
  UBRR0H = 0;
*/
  UCSR0B |= RXCIE0;
  restoreInterrupts(flags); // restore interrupt state
  Serial.flush();
  Serial.begin(9600);
  while(Serial.available()) Serial.read();

  digitalWrite(PDM_PIN, LOW);
  Serial.println("mesg: recover from PDM playback!");
}

void loop() {
  int buttonState = digitalRead(powerButtonPin);
  if (buttonState && buttonState != currentButtonState){
    currentWorkingState = !currentWorkingState;
    buttonState = 2; // power change print delay
  }

  currentButtonState = buttonState;

  if (currentWorkingState) {
    digitalWrite(powerLedPin, HIGH);

    int irButtonState = digitalRead(irReceiveButtonPin);
    if (irButtonState && irButtonState != currentIrButtonState){
      currentIrModeState = !currentIrModeState;
      irButtonState = 2; // irreceiver change print delay
      if (currentIrModeState) {
        IrReceiver.begin(irReceivePin, ENABLE_LED_FEEDBACK); // internal led feedback = on
      } else {
        IrReceiver.stop();
      }
    }
    currentIrButtonState = irButtonState;
    if (currentIrModeState)
      digitalWrite(irLedPin, HIGH);
    else
      digitalWrite(irLedPin, LOW);
    if (irButtonState == 2) { // irreceiver change print delay
      print_irreceiver_change();
      irWaintingCode = 0;
    }
  } else {
    digitalWrite(irLedPin, LOW); // LOW = 0.47V on Elegoo Uno R3 when VREF=5.06V
    if (currentIrModeState) {
      currentIrModeState = FALSE;
      print_irreceiver_change();
    }
    digitalWrite(powerLedPin, LOW);
  }
  if (buttonState == 2) // power change print delay
    print_power_change();

/*
  // print out the state of the button
  Serial.print("working state: "); 
  Serial.print(currentWorkingState);
  Serial.print(" // button state: "); 
  Serial.println(currentButtonState);
*/


  if (currentIrModeState) {
    if (IrReceiver.decode()) {
        struct IRData *irdata = &IrReceiver.decodedIRData;

        size_t len = (irdata->rawDataPtr->rawlen + 1) / 2;
        if (len <= 1)
            goto ir_end;

        Serial.print("mesg: ir code registration: ");
        if (irdata->flags & IRDATA_FLAGS_WAS_OVERFLOW) {
          Serial.println(F("received overflow"));
          return;
        }
        Serial.print(len, DEC);
        Serial.println(F(" bits (incl. gap and start) received"));

//        Serial.println(irdata->decodedRawData, HEX);
//        IrReceiver.printIRResultShort(&Serial);
//        IrReceiver.printIRSendUsage(&Serial);
//        IrReceiver.printIRResultRawFormatted(&Serial, true);

        tone(activeBuzzerPin, NOTE_C5, 1);

        int i = 0;
        for (; i < irdata->rawDataPtr->rawlen - 1; i++) {
//            Serial.println(EEPROM_IR_START + RAW_BUFFER_LENGTH * irWaintingCode + i);
            EEPROM.update(EEPROM_IR_START + RAW_BUFFER_LENGTH * irWaintingCode + i, irdata->rawDataPtr->rawbuf[i+1]); // see the comment of EEPROM specification of the device
        }
//        Serial.println(EEPROM_IR_START + RAW_BUFFER_LENGTH * irWaintingCode + i);
        EEPROM.update(EEPROM_IR_START + RAW_BUFFER_LENGTH * irWaintingCode + i, 0);

        delay(1000);
        if (irWaintingCode < IR_CODE_LEN - 1) {
            irWaintingCode ++;
            IrReceiver.resume();
        } else {
            currentIrModeState = FALSE;
            IrReceiver.stop();
        }
        print_irreceiver_change();
    }
  }
  ir_end:


  if (currentWorkingState) {
    if (Serial.available() > 0) {
      int cmd = Serial.read();
      if (cmd == 'a')
        pdm_playback_on_pin6();
    }


// 音楽を変えられるようにする？ EEPROMに保存する？ならす時間もEEPROMに保存する？デジタル時刻表示も欲しくなるけど過剰、かなぁ。
/*
// school chime
    tone(activeBuzzerPin, NOTE_F5, 500);
    delay(500);
    tone(activeBuzzerPin, NOTE_A5, 500);
    delay(500);
    tone(activeBuzzerPin, NOTE_G5, 500);
    delay(500);
    tone(activeBuzzerPin, NOTE_C5, 1500);
    delay(1500);

    tone(activeBuzzerPin, NOTE_F5, 500);
    delay(500);
    tone(activeBuzzerPin, NOTE_G5, 500);
    delay(500);
    tone(activeBuzzerPin, NOTE_A5, 500);
    delay(500);
    tone(activeBuzzerPin, NOTE_F5, 1500);
    delay(1500);

    tone(activeBuzzerPin, NOTE_A5, 500);
    delay(500);
    tone(activeBuzzerPin, NOTE_F5, 500);
    delay(500);
    tone(activeBuzzerPin, NOTE_G5, 500);
    delay(500);
    tone(activeBuzzerPin, NOTE_C5, 1500);
    delay(1500);

    tone(activeBuzzerPin, NOTE_C5, 500);
    delay(500);
    tone(activeBuzzerPin, NOTE_G5, 500);
    delay(500);
    tone(activeBuzzerPin, NOTE_A5, 500);
    delay(500);
    tone(activeBuzzerPin, NOTE_F5, 1500);
    delay(1500);*/

/*
    int direct3V3 = analogRead(direct3V3VoltagePin);

    Serial.print("d3V3: ");
    Serial.println(direct3V3);

    const float actual3V3 = 3.32;  // for Elegoo Uno R3; invariant across USB ports
    // https://forum.arduino.cc/t/dividing-by-1023-or-1024-the-final-verdict-on-analogread/516322
    float actualRef5V = actual3V3 * 1024 / (direct3V3+0.5);
    //    const int actualRef5V = 5.06;
    //    float actual3V3 = ((float)d3V3+0.5) * actualRef5V / 1024;
    //    Serial.print("actual3V3: ");
    //    Serial.println(actual3V3);
    Serial.print("ar5V: ");
    Serial.println(actualRef5V);
*/


    int praw = analogRead(photoresistorPin);
    Serial.print("praw: ");
    Serial.println(praw);

    int pirValue = digitalRead(pirPin);
    Serial.print("pir: "); // don't do four letter
    Serial.println(pirValue);

    if (pirValue && praw < 30) { // TODO: LUX閾値もEEPROMに保存する?
         Serial.println("callled!");
        ir_send(IR_LIGHTING_FULL);
    }


    float temperature;
    float humidity;

    if(measure_environment( &temperature, &humidity )){
      Serial.print("temp: ");
      Serial.println(temperature, 1);
      Serial.print("humi: ");
      Serial.println(humidity, 1);
    }

    delay(500);  // delay in between reads for stability
    return;
  }
  delay(500);  // delay in between reads for stability
}
