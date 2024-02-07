/*
Copyright (C) 2024 Toshimitsu Kimura <lovesyao@gmail.com>
SPDX-License-Identifier: Apache-2.0	
*/

#include <DHT_Async.h>
#define DHT_SENSOR_TYPE DHT_TYPE_11

static const int DHT_SENSOR_PIN = 7;

/*
Copyright (C) 2023 Toan Nguyen
SPDX-License-Identifier: Apache-2.0	
https://github.com/toannv17/DHT-Sensors-Non-Blocking/blob/main/examples/DHTAsyncTester/DHTAsyncTester.ino
*/
DHT_Async dht_sensor(DHT_SENSOR_PIN, DHT_SENSOR_TYPE);

static bool measure_environment(float *temperature, float *humidity) {
    static unsigned long measurement_timestamp = millis();

    if (millis() - measurement_timestamp > 2000ul) {
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
int photoresistorPin = 0; // A0
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
  // print out the state of the button:/home/nazo/Desktop/clock/gsclock_local/gsclock_arduino/gsclock_arduino_old.ino
  Serial.print("working state: "); 
  Serial.print(currentWorkingState);
  Serial.print(" // button state: "); 
  Serial.println(currentButtonState);
*/

  if (currentWorkingState){
    digitalWrite(powerLedPin, HIGH);
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
