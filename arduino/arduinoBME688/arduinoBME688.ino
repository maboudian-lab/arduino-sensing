/*
  Isaac Zakaria
  Gordon Guo
  1 November 2021
  Rev: 3 February 2022
  Board: Adafruit Feather Adalogger M0

  Monitor voltages from pins A0, A1, A2, A3, A6, A7 with moving-average filter
  Monitor BME688 RH, temperature, and VOC sensor resistance
  Monitor SCD30 RH, temperature, and NDIR CO2 concentration
  [Pins A4 (SDA), A5 (SCL) are skipped when reading voltages]
*/

/*
   LIBRARIES
*/

#include <Wire.h>
#include <SPI.h>

// Adafruit Bosch BME RH and T sensor (also senses P, but not used here)
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"

// Adafruit Sensirion SCD-30 NDIR CO2, RH, and T sensor
//#include <Adafruit_SCD30.h>

// OLED
//#include <Adafruit_GFX.h>
//#include <Adafruit_SH110X.h>

/*
   USER-SPECIFIED PARAMETERS
*/

// Baud rate
#define baudRate 115200

// Delay time between e-nose measurements (ms)
#define delayTime 5



/*
   ADAFRUIT SENSOR SETUP
*/

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

// Initialize sensors over I2C
Adafruit_BME680 bme;


float bmeRh;
float bmeTemp;
float bmeVoc;

bool bmeFlag;

//unsigned long delayTime;

// Figaro sensor readout with moving-average filter

#define windowSize 100

#define denominator 10000

#define len 199

// Array to hold raw analog pin readout
int analogOut[len][8];
// Array to hold voltages
float vOut[8];
// Conversion factor from bits to mV
float conversion = 3300.0 / 4095.0;
// Loop index for moving-average filter
int i = 0;
// Array of analog pin indices
int pinIndex[6] = {0, 1, 2, 3, 6, 7};

unsigned long timeOfEst = 0;

int weights[len];

void setup()
{

  Serial.begin(baudRate);
  while (!Serial)
    ; // Wait for serial connection to begin taking readings



  bmeFlag = bme.begin();
  // Check for BME680
  if (!bmeFlag)
  {
    Serial.println("No BME680 sensor found!");
  }
  else
  {
    // Set up oversampling and filter initialization
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150); // 320*C for 150 ms
  }
  if (!bmeFlag)
  {
    while (1)
      ;
  }

  analogReadResolution(12); // 12-bit ADC

  for (int i = 0; i < windowSize; i++)
  {
    weights[i] = i + 1;
    weights[len - 1 - i] = i + 1;
  }
}

void loop()
{

  if (i == (len - 1))
  {
    Serial.println("x");
  }

  for (int j = 0; j < 6; j++)
  {
    analogOut[i][j] = analogRead(pinIndex[j]); // read new analog values and overwrite old value outside of windowSize
    vOut[j] += weights[i] * analogOut[i][j];   // update moving-average filter sum

    if (i == (len - 1))
    { // output sensor voltages after every windowSize readings

      vOut[j] = (vOut[j] / denominator) * conversion;
      Serial.print(pinIndex[j]);
      Serial.print("  ");
      Serial.println(vOut[j]);
      vOut[j] = 0;
    }
  }

  if (i == (len - 1))
  { // output RH and temperature after every windowSize readings

    bmeRh = bme.readHumidity();
    bmeTemp = bme.readTemperature();
    bmeVoc = bme.gas_resistance;

    // print to serial
    Serial.print("HB ");
    Serial.println(bmeRh);

    Serial.print("TB ");
    Serial.println(bmeTemp);

    Serial.print("RB ");
    Serial.println(bmeVoc);

    timeOfEst = millis();
    Serial.print("t  ");
    Serial.println(timeOfEst);
  }

  if (i == (len - 1))
  {
    Serial.println("y");
  }

  i = (i + 1) % len;

  // delay [ms]
  delay(delayTime);
}
