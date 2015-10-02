# SmartwatchV1
First version of my smartwatch project. Very early build.
It works with the Atmega328P-PU on power down mode. Brownout detector off (fuses off), and:
- Oled I2C display (SSD1306);
- MPU6050 Accelerometer and gyro;
- HMC5883L compass;
- BMP180 pressure sensor;
- PCF8563 real-time clock;
- AT24C64 EEPROM memory;

The user must get only below 120uA with all the sensors and display sleeping. It is possible to get even lower, depending on your hardware. Without the MPU6050, HMC5883L and BMP180, the other pieces must use only 1.1uA. Please download also the other libraries:

#include <LowPower.h>
#include <I2Cdev.h> 
#include <MPU6050.h>
#include <U8glib.h> 
#include <HMC5883L.h> 
#include <Adafruit_BMP085.h> 

It also uses:

#include <Wire.h> //comes with the Arduino IDE
#include <avr/sleep.h> //comes with the Arduino IDE
#include <avr/power.h> //comes with the Arduino IDE
#include <avr/wdt.h> //comes with the Arduino IDE
#include <EEPROM.h> //comes with the Arduino IDE
