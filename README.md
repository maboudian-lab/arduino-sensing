# arduino-sensing
Arduino and Python codes for monitoring sensors and recording data.

### Current boards in use:
- Arduino Nano 33 BLE
- Adafruit Feather M0 Adalogger
- Adafruit nRF52840 Feather (BLE)

### Arduino dependencies:
#### Boards
- Arduino Mbed OS Nano Boards (Arduino Nano 33 BLE support)
- Adafruit SAMD Boards (Adafruit Feather M0 support)
- Adafruit nRF52 (Adafruit nRF52840 Feather support)

Add Adafruit boards to Arduino IDE by adding

`https://adafruit.github.io/arduino-board-index/package_adafruit_index.json`

as an Additional Board Manager URL under Preferences.

#### Libraries
- Adafruit Feather OLED (126x64 OLED display support)
- Adafruit BME680 (Adafruit Bosch BME680/688 support)
- Adafruit SCD30 (Adafruit Sensirion SCD30 support)

### Python dependencies:
- pySerial
- csv
- NumPy

NumPy and csv should come bundled with a standard Anaconda installation.

#### Hardware setup
Connect all commercial sensors via I2C. For Adafruit STEMMA QT cables:
- Yellow: SCL
- Blue: SDA
- Red: 3.3 V or 5 V
- Black: GND

### BME280/680:
![BME280 connections for I2C.](https://cdn-learn.adafruit.com/assets/assets/000/097/118/original/adafruit_products_BME280_arduino_I2C_STEMMA_bb.jpg?1605726553)
https://cdn-shop.adafruit.com/product-files/3660/BME680.pdf documentation regarding the BME 680 Sensor component 

### SCD30:
![SCD30 connections for I2C.](https://cdn-learn.adafruit.com/assets/assets/000/098/479/original/adafruit_products_SCD30_arduino_I2C_STEMMA_bb.jpg?1609893467)
