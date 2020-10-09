# Shot Counter
Inspired by https://github.com/marc-schieferdecker/arduino-shot-counter.

## Overview

**An Arduino gun mountable shot counter**

Hardware components used:
* Arduino Board Nano ATMega328
* Gyro GY-521 using MPU6050

## Wireing

Arduino
* Attach a power source to VIN (+) and GND (-)
* (or use usb power for testing)

MPU6050
* GND and AD0 -> Arduino GND
* VCC -> Arduino 5V
* SDA -> Arduino A5
* SCL -> Arduino A4

## Future Development
* set all Serial.prints inside a DEBUG to speed up processing and code size in the Arduino
* find proper shot detection thresholds
* add microphon for better shot detection to be used with acc and gyro
* add bluetooth to communicate with mobile app
