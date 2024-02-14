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
  pinMode(powerButtonPin, INPUT);
  pinMode(powerLedPin, OUTPUT);
  pinMode(irReceiveButtonPin, INPUT);
  pinMode(irLedPin, OUTPUT);
  IrSender.begin(irSendPin);
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

// working but comment out for now
/*
    if (praw < 30) { // TODO: 人感センサーも有効にする、LUX閾値もEEPROMに保存する?
         Serial.println("callled!");
        ir_send(IR_LIGHTING_FULL);
    }
*/

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
