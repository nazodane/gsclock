/*
Copyright (C) 2024 Toshimitsu Kimura <lovesyao@gmail.com>
SPDX-License-Identifier: Apache-2.0
*/

#include "DHT-Sensors-Non-Blocking/DHT_Async.cpp"
#include "Tone/Tone.h"
//#include "Arduino-IRremote/src/IRReceive.hpp" # or IRMP?
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
int photoresistorPin = 0; // A0
int direct3V3VoltagePin = 5;  // A5
int currentButtonState = FALSE;

//bool currentWorkingState = FALSE;
bool currentWorkingState = TRUE;

void setup() {
  Serial.begin(9600);
  pinMode(powerButtonPin, INPUT);
  pinMode(powerLedPin, OUTPUT);
}

void loop() {
  int buttonState = digitalRead(powerButtonPin);
  if(buttonState && buttonState != currentButtonState){
      currentWorkingState = !currentWorkingState;
  }
  currentButtonState = buttonState;

/*
  // print out the state of the button
  Serial.print("working state: "); 
  Serial.print(currentWorkingState);
  Serial.print(" // button state: "); 
  Serial.println(currentButtonState);
*/

  if (currentWorkingState){
    digitalWrite(powerLedPin, HIGH);

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
  digitalWrite(powerLedPin, LOW);
  // led off  
  delay(500);  // delay in between reads for stability
}
