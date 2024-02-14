/*
Copyright (C) 2024 Toshimitsu Kimura <lovesyao@gmail.com>
SPDX-License-Identifier: Apache-2.0
*/

#include "DHT-Sensors-Non-Blocking/DHT_Async.cpp"
#include "Tone/Tone.h"

#define IR_USE_AVR_TIMER1 // timer1 for ir recv and timer2 for tone
#define RAW_BUFFER_LENGTH 100 // just in case
#define NO_DECODER // just resend the raw data so the decoders are not needed
#include "Arduino-IRremote/src/IRremote.hpp" // or IRMP?
//#include <EEPROM.h> # save the IR remote control code for home lighting. lowest on/off/up/down?


// EEPROM[0..4] = GS00
// EEPROM[5] = LIGHTING FULL LIGHT IR COMMAND SIZE
// EEPROM[6] = LIGHTING NIGHT LIGHT IR COMMAND SIZE
// EEPROM[7] = LIGHTING LIGHT OUT IR COMMAND SIZE
// EEPROM[8] = LIGHTING LIGHTER IR COMMAND SIZE
// EEPROM[9] = LIGHTING DARKER IR COMMAND SIZE
// EEPROM[1023 - EEPROM[5]..1023] = LIGHTING FULL LIGHT IR COMMAND
// EEPROM[1023 - sum(EEPROM[5..6])..1023 - EEPROM[5]] = LIGHTING NIGHT LIGHT IR COMMAND
// EEPROM[1023 - sum(EEPROM[5..7])..1023 - sum(EEPROM[5..6])] = LIGHTING LIGHT OUT IR COMMAND
// EEPROM[1023 - sum(EEPROM[5..8])..1023 - sum(EEPROM[5..7])] = LIGHTING LIGHTER IR COMMAND
// EEPROM[1023 - sum(EEPROM[5..9])..1023 - sum(EEPROM[5..8])] = LIGHTING DARKER IR COMMAND

const char *IR_COMMAND_NAMES[] = { "full light", "night light",  "light out", "lighter", "darker"};
#define IR_COMMAND_LEN sizeof(IR_COMMAND_NAMES) / sizeof(IR_COMMAND_NAMES[0])

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
int activeBuzzerPin = 8; //D8
int irReceiveButtonPin = 10; //D10
int irLedPin = 9; // D9
int irReceivePin = 5;  // D5
int irSendPin = 3; // D3
int photoresistorPin = 0; // A0
int direct3V3VoltagePin = 5;  // A5
int currentButtonState = FALSE;
int currentIrButtonState = FALSE;

//bool currentWorkingState = FALSE;
bool currentWorkingState = TRUE;

bool currentIrModeState = FALSE;
size_t irWaintingCommand = 0;

void print_power_change() {
  Serial.print("mesg: power ");
  Serial.println(currentWorkingState?"on":"off");
}

void print_irreceiver_change() {
  Serial.print("mesg: ir code registration ");
  Serial.print(currentIrModeState?"on; waiting ":"off");

  if (currentIrModeState) {
    Serial.print(IR_COMMAND_NAMES[irWaintingCommand]);
    Serial.println(" code...");
  } else if (irWaintingCommand == IR_COMMAND_LEN - 1)
    Serial.println("; the registration succeeded!");
  else
    Serial.println();
}

void setup() {
  Serial.begin(9600);
  pinMode(powerButtonPin, INPUT);
  pinMode(powerLedPin, OUTPUT);
  pinMode(irReceiveButtonPin, INPUT);
  pinMode(irLedPin, OUTPUT);
  print_power_change();
}


unsigned int rawbuf_for_send[RAW_BUFFER_LENGTH];

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
      irWaintingCommand = 0;
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

        for (int i = 0; i < irdata->rawDataPtr->rawlen - 1; i++)
            rawbuf_for_send[i] = irdata->rawDataPtr->rawbuf[i+1] * MICROS_PER_TICK;

        rawbuf_for_send[irdata->rawDataPtr->rawlen - 1] = 0;

        delay(1000);
        if (irWaintingCommand < IR_COMMAND_LEN - 1) {
            irWaintingCommand ++;
            IrReceiver.resume();
        } else {
            currentIrModeState = FALSE;
            IrReceiver.stop();
        }
        print_irreceiver_change();

//        delay(500);
//        IrSender.sendRaw(rawbuf_for_send, irdata->rawDataPtr->rawlen, MICROS_PER_TICK / *kHZ* /);
    }
  }
  ir_end:


  if (currentWorkingState) {


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

// 送信するIRデータはEEPROMに保存する
// LUX閾値もEEPROMに保存する



// 人感センサー試す

    int praw = analogRead(photoresistorPin);
    Serial.print("praw: ");
    Serial.println(praw);

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
