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

- LowPower
- I2Cdev
- MPU6050
- U8glib
- HMC5883L
- Adafruit_BMP085

It also uses:

- Wire //comes with the Arduino IDE
- avr/sleep //comes with the Arduino IDE
- avr/power //comes with the Arduino IDE
- avr/wdt //comes with the Arduino IDE
- EEPROM //comes with the Arduino IDE
