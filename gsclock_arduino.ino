/*
Copyright (C) 2024 Toshimitsu Kimura <lovesyao@gmail.com>
SPDX-License-Identifier: Apache-2.0
*/

#include "DHT-Sensors-Non-Blocking/DHT_Async.cpp"
#include "Tone/Tone.h"

#define IR_USE_AVR_TIMER1 // timer1 for tone, timer2 for ir recv
#include "Arduino-IRremote/src/IRremote.hpp" // or IRMP?
//#include <EEPROM.h> # save the IR remote control code for home lighting. lowest on/off/up/down?

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
int irRecieveButtonPin = 10; //D10
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

void setup() {
  Serial.begin(9600);
  pinMode(powerButtonPin, INPUT);
  pinMode(powerLedPin, OUTPUT);
  pinMode(irRecieveButtonPin, INPUT);
  pinMode(irLedPin, OUTPUT);
}

void loop() {
  int buttonState = digitalRead(powerButtonPin);
  if (buttonState && buttonState != currentButtonState){
    currentWorkingState = !currentWorkingState;
  }

  currentButtonState = buttonState;

  if (currentWorkingState) {
    digitalWrite(powerLedPin, HIGH);

    int irButtonState = digitalRead(irRecieveButtonPin);
    if (irButtonState && irButtonState != currentIrButtonState){
      currentIrModeState = !currentIrModeState;
      if (currentWorkingState){
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
  } else {
    digitalWrite(irLedPin, LOW); // LOW = 0.47V on Elegoo Uno R3 VREF=5.06V
    currentIrModeState = FALSE;
    digitalWrite(powerLedPin, LOW);
  }

/*
  // print out the state of the button
  Serial.print("working state: "); 
  Serial.print(currentWorkingState);
  Serial.print(" // button state: "); 
  Serial.println(currentButtonState);
*/

  if (currentWorkingState){

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

/*
    if (IrReceiver.decode()) {
        struct IRData *irdata = &IrReceiver.decodedIRData;
        Serial.println(irdata->decodedRawData, HEX); // Print "old" raw data
        // USE NEW 3.x FUNCTIONS
        IrReceiver.printIRResultShort(&Serial); // Print complete received data in one line
        IrReceiver.printIRSendUsage(&Serial);   // Print the statement required to send this data
        IrReceiver.printIRResultRawFormatted(&Serial, true);
        dump(irdata->rawDataPtr);
        //tone(activeBuzzerPin, NOTE_C5, 1);
        delay(500);
        IrReceiver.resume(); // Enable receiving of the next value
        delay(500);
//        Serial.println(irdata->rawDataPtr->rawlen* sizeof(*irdata->rawDataPtr->rawbuf));
// https://forum.arduino.cc/t/problems-with-irsend-sendraw-sending-raw-ir-code/166479

        for (int i = 0; i < irdata->rawDataPtr->rawlen - 1; i++)
            rawbuf_for_send[i] = irdata->rawDataPtr->rawbuf[i+1] * MICROS_PER_TICK;

        rawbuf_for_send[irdata->rawDataPtr->rawlen - 1] = 0;
//        Serial.println(sizeof(int) == sizeof(*irdata->rawDataPtr->rawbuf));
        IrSender.sendRaw(rawbuf_for_send, irdata->rawDataPtr->rawlen, MICROS_PER_TICK / *kHZ* /);
    }
*/


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
