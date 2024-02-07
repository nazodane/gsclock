/*
Copyright (C) 2024 Toshimitsu Kimura <lovesyao@gmail.com>
SPDX-License-Identifier: Apache-2.0
*/


//#include <DHT_Async.h>

/*
Copyright (C) 2023 Toan Nguyen
SPDX-License-Identifier: Apache-2.0
copy from https://github.com/toannv17/DHT-Sensors-Non-Blocking/blob/a195a0a05c314923bdb528e61b4ef2672a022b55/DHT_Async.h
*/
#ifndef _DHT_ASYNC_H_
#define _DHT_ASYNC_H_

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif


#define DHT_TYPE_11  0
#define DHT_TYPE_12  1
#define DHT_TYPE_21  2
#define DHT_TYPE_22  3


class DHT_Async {
public:
    DHT_Async(uint8_t pin, uint8_t type);

    bool measure(float *temperature, float *humidity, bool autoSync = true);

    void measureSync(float *temperature, float *humidity);

    float convertCtoF(float c);

    float convertFtoC(float f);

private:
    uint8_t dhtState;
    unsigned long dhtTimestamp;
    uint8_t data[5];
    const uint8_t _pin, _type;
#ifdef __AVR
    // Use direct GPIO access on an 8-bit AVR so keep track of the port and
    // bitmask for the digital pin connected to the DHT.  Other platforms will use
    // digitalRead.
    const uint8_t _bit, _port;
#endif
    const uint32_t _maxCycles;

    bool readData();

    bool readAsync();

    float readTemperature() const;

    float readHumidity() const;

    uint32_t expectPulse(bool level) const;
};


class DHT_Async_Interrupt {
public:
    DHT_Async_Interrupt() {
#if !defined(ARDUINO_ARCH_NRF52)
        noInterrupts();
#endif
    }

    ~DHT_Async_Interrupt() {
#if !defined(ARDUINO_ARCH_NRF52)
        interrupts();
#endif
    }
};

#endif //_DHT_ASYNC_H_

/*
Copyright (C) 2023 Toan Nguyen
SPDX-License-Identifier: Apache-2.0
based on https://github.com/toannv17/DHT-Sensors-Non-Blocking/a195a0a05c314923bdb528e61b4ef2672a022b55/main/DHT_Async.cpp
*/
//#include "DHT_Async.h" // XXX: modified by gsclock

#define DHT_IDLE                  0
#define DHT_BEGIN_MEASUREMENT     1
#define DHT_BEGIN_MEASUREMENT_2   2
#define DHT_DO_READING            3
#define DHT_COOLDOWN              4


/* Number of milliseconds before a new sensor read may be initiated. */
#define COOLDOWN_TIME  2000


/*
 * Constructor for the sensor.  It remembers the pin number and the
 * type of sensor, and initializes internal variables.
 */
DHT_Async::DHT_Async(uint8_t pin, uint8_t type)
        : _pin(pin),
          _type(type),
#ifdef __AVR
          _bit(digitalPinToBitMask(pin)),
          _port(digitalPinToPort(pin)),
#endif
          _maxCycles(microsecondsToClockCycles(1000)) {
    dhtState = DHT_IDLE;

    pinMode(_pin, INPUT);
    digitalWrite(_pin, HIGH);
}

/*
 * Instruct the DHT to begin sampling.  Keep polling until it returns true.
 * The temperature is in degrees Celsius, and the humidity is in %.
 */
bool DHT_Async::measure(float *temperature, float *humidity, bool autoSync) {
    if (autoSync && (
            (dhtState == DHT_BEGIN_MEASUREMENT_2 && millis() - dhtTimestamp > 500) ||
            (dhtState == DHT_DO_READING && millis() - dhtTimestamp > 90)
        )
    ) {
        measureSync(temperature, humidity);
        return true;
    }

    if (readAsync()) {
        *temperature = readTemperature();
        *humidity = readHumidity();
        return true;
    } else {
        return false;
    }
}

void DHT_Async::measureSync(float *temperature, float *humidity) {
    while (1) {
        if (readAsync()) {
            *temperature = readTemperature();
               *humidity = readHumidity();
            return;
        }
    }
}

float DHT_Async::readTemperature() const {
    float f = NAN;

    switch (_type) {
        case DHT_TYPE_11:
            f = data[2];
            if (data[3] & 0x80) {
                f = -1 - f;
            }
            f += (data[3] & 0x0f) * 0.1;
            break;

        case DHT_TYPE_12:
            f = data[2];
            f += (data[3] & 0x0f) * 0.1;
            if (data[2] & 0x80) {
                f *= -1;
            }
            break;

        case DHT_TYPE_21:
        case DHT_TYPE_22:
            f = ((word) (data[2] & 0x7F)) << 8 | data[3];
            f *= 0.1;
            if (data[2] & 0x80) {
                f *= -1;
            }
            break;
    }

    return f;
}


float DHT_Async::readHumidity() const {
    float f = NAN;

    switch (_type) {
        case DHT_TYPE_11:
        case DHT_TYPE_12:
            f = data[0] + data[1] * 0.1;
            break;

        case DHT_TYPE_21:
        case DHT_TYPE_22:
            f = ((word) data[0]) << 8 | data[1];
            f *= 0.1;
            break;
    }

    return f;
}


/*
 * Expect the input to be at the specified level and return the number
 * of loop cycles spent there.  This is identical to Adafruit's blocking
 * driver.
 */
uint32_t DHT_Async::expectPulse(bool level) const {
// F_CPU is not be known at compile time on platforms such as STM32F103.
// The preprocessor seems to evaluate it to zero in that case.
#if (F_CPU > 16000000L) || (F_CPU == 0L)
    uint32_t count = 0;
#else
    uint16_t count = 0; // To work fast enough on slower AVR boards
#endif
    // On AVR platforms use direct GPIO port access as it's much faster and better
    // for catching pulses that are 10's of microseconds in length:
#ifdef __AVR
    uint8_t portState = level ? _bit : 0;
    while ((*portInputRegister(_port) & _bit) == portState) {
        if (count++ >= _maxCycles) {
            return 0; // Exceeded timeout, fail.
        }
    }
    // Otherwise fall back to using digitalRead (this seems to be necessary on ESP8266
    // right now, perhaps bugs in direct port access functions?).
#else
    while (digitalRead(_pin) == level) {
        if (count++ >= _maxCycles) {
            return 0; // Exceeded timeout, fail.
        }
    }
#endif

    return count;
}


/*
 * State machine of the non-blocking read.
 */
bool DHT_Async::readAsync() {
    bool status = false;

    switch (dhtState) {
        /* We may begin measuring any time. */
        case DHT_IDLE:
            dhtState = DHT_BEGIN_MEASUREMENT;
            break;

            /* Initiate a sensor read.  The read begins by going to high impedance
               state for 250 ms. */
        case DHT_BEGIN_MEASUREMENT:
            digitalWrite(_pin, HIGH);
            /* Reset 40 bits of received data to zero. */
            data[0] = data[1] = data[2] = data[3] = data[4] = 0;
            dhtTimestamp = millis();
            dhtState = DHT_BEGIN_MEASUREMENT_2;
            break;

            /* After the high impedance state, pull the pin low for 20 ms. */
        case DHT_BEGIN_MEASUREMENT_2:
            /* Wait for 250 ms. */
            if (millis() - dhtTimestamp > 250) {
                pinMode(_pin, OUTPUT);
                digitalWrite(_pin, LOW);
                dhtTimestamp = millis();
                dhtState = DHT_DO_READING;
            }
            break;

        case DHT_DO_READING:
            /* Wait for 20 ms. */
            if (millis() - dhtTimestamp > 20) {
                dhtTimestamp = millis();
                dhtState = DHT_COOLDOWN;
                status = readData();
//      if( status != true )
//      {
//        Serial.println( "Reading failed" );
//      }
            }
            break;

            /* If it has been less than two seconds since the last time we read
               the sensor, then let the sensor cool down.. */
        case DHT_COOLDOWN:
            if (millis() - dhtTimestamp > COOLDOWN_TIME) {
                dhtState = DHT_IDLE;
            }
            break;

        default:
            break;
    }

    return status;
}


/* Read sensor data.  This is identical to Adafruit's blocking driver. */
bool DHT_Async::readData() {
    uint32_t cycles[80];

    /* Turn off interrupts temporarily because the next sections are timing critical
       and we don't want any interruptions. */
    {
        volatile DHT_Async_Interrupt interrupt;

        // End the start signal by setting data line high for 40 microseconds.
        digitalWrite(_pin, HIGH);
        delayMicroseconds(40);

        // Now start reading the data line to get the value from the DHT sensor.
        pinMode(_pin, INPUT);
        // Delay a bit to let sensor pull data line low.
        delayMicroseconds(10);

        // First expect a low signal for ~80 microseconds followed by a high signal
        // for ~80 microseconds again.
        if (expectPulse(LOW) == 0) {
            return false;
        }
        if (expectPulse(HIGH) == 0) {
            return false;
        }

        // Now read the 40 bits sent by the sensor.  Each bit is sent as a 50
        // microsecond low pulse followed by a variable length high pulse.  If the
        // high pulse is ~28 microseconds then it's a 0 and if it's ~70 microseconds
        // then it's a 1.  We measure the cycle count of the initial 50us low pulse
        // and use that to compare to the cycle count of the high pulse to determine
        // if the bit is a 0 (high state cycle count < low state cycle count), or a
        // 1 (high state cycle count > low state cycle count). Note that for speed all
        // the pulses are read into a array and then examined in a later step.
        for (int i = 0; i < 80; i += 2) {
            cycles[i] = expectPulse(LOW);
            cycles[i + 1] = expectPulse(HIGH);
        }

        /* Timing critical code is now complete. */
    }

    // Inspect pulses and determine which ones are 0 (high state cycle count < low
    // state cycle count), or 1 (high state cycle count > low state cycle count).
    for (int i = 0; i < 40; ++i) {
        uint32_t low_cycles = cycles[2 * i];
        uint32_t high_cycles = cycles[2 * i + 1];
        if ((low_cycles == 0) || (high_cycles == 0)) {
            return false;
        }
        data[i / 8] <<= 1;
        // Now compare the low and high cycle times to see if the bit is a 0 or 1.
        if (high_cycles > low_cycles) {
            // High cycles are greater than 50us low cycle count, must be a 1.
            data[i / 8] |= 1;
        }
        // Else high cycles are less than (or equal to, a weird case) the 50us low
        // cycle count so this must be a zero.  Nothing needs to be changed in the
        // stored data.
    }

    // Check we read 40 bits and that the checksum matches.
    if (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) {
        return true;
    } else {
        return false;
    }
}

/*!
 *  @brief  Converts Celsius to Fahrenheit
 *  @param  c value in Celsius
 *	@return float value in Fahrenheit
 */
float DHT_Async::convertCtoF(float c) { return c * 1.8 + 32; }

/*!
 *  @brief  Converts Fahrenheit to Celsius
 *  @param  f value in Fahrenheit
 *	@return float value in Celsius
 */
float DHT_Async::convertFtoC(float f) { return (f - 32) * 0.55555; }



/*
Unique code for gsclock
*/
#define DHT_SENSOR_TYPE DHT_TYPE_11

static const int DHT_SENSOR_PIN = 7;

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
  // print out the state of the button
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
