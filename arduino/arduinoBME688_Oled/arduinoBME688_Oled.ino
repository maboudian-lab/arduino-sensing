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
 * LIBRARIES
 */

#include <Wire.h>
#include <SPI.h>

// Adafruit Bosch BME RH and T sensor (also senses P, but not used here)
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"

// Adafruit Sensirion SCD-30 NDIR CO2, RH, and T sensor
#include <Adafruit_SCD30.h>

// OLED
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

/*
 * USER-SPECIFIED PARAMETERS
 */

// Baud rate
#define baudRate 115200

// Delay time between e-nose measurements (ms)
#define delayTime 5

/*
 * ADAFRUIT FEATHERWING 64x128 OLED SETUP
 */

// 64x128 OLED FeatherWing
Adafruit_SH1107 display = Adafruit_SH1107(64, 128, &Wire);

// OLED FeatherWing buttons map to different pins depending on board:
#if defined(ESP8266)
#define BUTTON_A 0
#define BUTTON_B 16
#define BUTTON_C 2
#elif defined(ESP32) && !defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2)
#define BUTTON_A 15
#define BUTTON_B 32
#define BUTTON_C 14
#elif defined(ARDUINO_STM32_FEATHER)
#define BUTTON_A PA15
#define BUTTON_B PC7
#define BUTTON_C PC5
#elif defined(TEENSYDUINO)
#define BUTTON_A 4
#define BUTTON_B 3
#define BUTTON_C 8
#elif defined(ARDUINO_NRF52832_FEATHER)
#define BUTTON_A 31
#define BUTTON_B 30
#define BUTTON_C 27
#else // 32u4, M0, M4, nrf52840, esp32-s2 and 328p
#define BUTTON_A 9
#define BUTTON_B 6
#define BUTTON_C 5
#endif

/*
 * ADAFRUIT SENSOR SETUP
 */

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

// Initialize sensors over I2C
Adafruit_SCD30 scd30;
Adafruit_BME680 bme;


float bmeRh;
float bmeTemp;
float bmeVoc;
float scdRh;
float scdTemp;
float scdCO2;

bool bmeFlag;
bool scdFlag;

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

  // Initialize OLED
  display.begin(0x3C, true); // Address 0x3C default
  display.display();         // Show image buffer
  delay(250);

  // Clear the buffer.
  display.clearDisplay();
  display.display();

  display.setRotation(1);

  // Set button functions for debug purposes
  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);

  // Initial text display settings
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);

  display.println("PROPERTY OF\n MABOUDIAN LAB >:D");
  display.display();
  delay(500);

  display.clearDisplay();
  display.display();
  display.println("Checking for sensors");
  display.display();

  bmeFlag = bme.begin();
  scdFlag = scd30.begin();

  // Check for BME680
  if (!bmeFlag)
  {
    Serial.println("No BME680 sensor found!");
    display.println("No BME680 found :(");
  }
  else
  {
    display.println("BME680 found! :)");
    // Set up oversampling and filter initialization
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150); // 320*C for 150 ms
  }
  // Check for SCD-30
  if (!scdFlag)
  {
    Serial.println("No SCD-30 sensor found!");
    display.println("No SCD-30 found :(");
  }
  else
  {
    display.println("SCD-30 found! :)");
  }
  display.display();
  if (!bmeFlag || !scdFlag)
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
  //OLED buttons for debug purposes
  //  if(!digitalRead(BUTTON_A)) display.print("A");
  //  if(!digitalRead(BUTTON_B)) display.print("B");
  //  if(!digitalRead(BUTTON_C)) display.print("C");

    if (i == (windowSize - 1))
    {
      Serial.println("x");
      timeOfEst = millis();
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

    scd30.read();

    bmeRh = bme.readHumidity();
    scdRh = scd30.relative_humidity;
    bmeTemp = bme.readTemperature();
    scdTemp = scd30.temperature;
    bmeVoc = bme.gas_resistance;
    scdCO2 = scd30.CO2;

    // print to serial
    Serial.print("HB ");
    Serial.println(bmeRh);

    Serial.print("TB ");
    Serial.println(bmeTemp);

    Serial.print("RB ");
    Serial.println(bmeVoc);

    Serial.print("HS ");
    Serial.println(scdRh);

    Serial.print("TS ");
    Serial.println(scdTemp);

    Serial.print("CS ");
    Serial.println(scdCO2);

    Serial.print("t  ");
    Serial.println(timeOfEst);

    // print to display
    display.clearDisplay();
    display.setCursor(0, 0);

    display.print("Uptime: ");
    display.print(timeOfEst / 1000.0);
    display.println(" s");

    display.println("BME: ");
    display.print(bmeRh);
    display.print(" %RH | ");
    display.print(bmeTemp);
    display.print(" C | ");
    display.println(bmeVoc);

    display.println("SCD: ");
    display.print(scdRh);
    display.print(" %RH | ");
    display.print(scdTemp);
    display.print(" C ");
    display.println(scdCO2);


    display.display();
  }

  i = (i + 1) % len;

  // delay [ms]
  delay(delayTime);
}
