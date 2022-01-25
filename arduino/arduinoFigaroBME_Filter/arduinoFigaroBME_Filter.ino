 /*
 Isaac Zakaria
 Gordon Guo
 1 November 2021
 Rev: 22 November 2021
 Board: Arduino Nano 33 BLE
 
 Monitor voltages from pins A0, A1, A2, A3, A6, A7 with moving-average filter
 Monitor BME280 RH and temperature
 [Pins A4 (SDA), A5 (SCL) are skipped when reading voltages]
 */

// BME280 readout from bme280test.ino
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C
//Adafruit_BME280 bme(BME_CS); // hardware SPI
//Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI

unsigned long delayTime;

// Figaro sensor readout with moving-average filter

# define windowSize 100

# define denominator 10000

# define len 199

// Array to hold raw analog pin readout
int analogOut[len][8];
// Array to hold voltages
float vOut[8];
// Conversion factor from bits to mV
float conversion = 3300.0/4095.0;
// Loop index for moving-average filter
int i = 0;
// Array of analog pin indices
int pinIndex[6] = {0, 1, 2, 3, 6, 7};

int timeOfEst = 0;

int weights[len];

void setup() {
    Serial.begin(9600);
    while(!Serial);    // time to get serial running
//    Serial.println(F("BME280 test"));

    unsigned status;
    
    // default settings
    status = bme.begin();  
    // You can also pass in a Wire library object like &Wire2
    // status = bme.begin(0x76, &Wire2)
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
        while (1) delay(10);
    }
    
//    Serial.println("-- Default Test --");
//    delayTime = 1000;
    delayTime = 5;

//    Serial.println();
    analogReadResolution(12); // 12-bit ADC

    for (int i = 0; i < windowSize; i++) {
        weights[i] = i+1;
        weights[len-1-i] = i+1;
    }
}

void loop() {
  for (int j = 0; j < 6; j++) {
    analogOut[i][j] = analogRead(pinIndex[j]); // read new analog values and overwrite old value outside of windowSize
    vOut[j] += weights[i] * analogOut[i][j]; // update moving-average filter sum

    if (i == (len-1)) { // output sensor voltages after every windowSize readings
      vOut[j] = (vOut[j]/denominator)*conversion;
      // Serial.print("yay");
      Serial.print(pinIndex[j]);
      Serial.print(" ");
      Serial.println(vOut[j]);
      vOut[j] = 0;
    }
  }

  if (i == (windowSize-1)) {
    timeOfEst = millis();
  }

  if (i == (len-1)) { // output temperature and RH after every windowSize readings
    Serial.print("H ");
    Serial.println(bme.readHumidity());
    
    Serial.print("T ");
    Serial.println(bme.readTemperature());

//    Serial.println();

    Serial.print("t ");
    Serial.println(timeOfEst);
  }

  i = (i+1) % len;

  // delay [ms]
  delay(delayTime);
}
