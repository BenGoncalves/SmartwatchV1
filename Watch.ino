//INCLUDES
#include <LowPower.h>
#include <Wire.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <I2Cdev.h> //nothing to do 
#include <MPU6050.h>
#include <HMC5883L.h> //ok power down mode
#include <Adafruit_BMP085.h> //stad-by mode
#include <U8glib.h> //sleep mode
#include <EEPROM.h>

//GLOBAL
#define LED_PIN 13

//variables
Adafruit_BMP085 bmp;
HMC5883L compass;
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NO_ACK); //U8G_I2C_OPT_NONE

//BUTTON
#define BUTTON 2
#define NUMCOUNTB1 7
volatile bool buttonInterrupt = false;
unsigned int BUTTON1 = 1;

//COMPASS
double error;
int16_t mx, my, mz;

//DISPLAY
#define BRIGHTNESS 0x01
#define BRIGHTNESSREG 0x81
unsigned long int timer_draw = 0;
String str1;  //declaring string

//TIME
#define PCF8563address 0x51
byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;
String days[] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday" };

//EEPROM
int addr = 2;
#define addStp 0 //address of byte to store number of steps
#define addCal 4 //address of byte to store number of calories 
#define addKm 8 //address of byte to store number of km 

//ACCELEROMETER
#define ACCINTPIN 3
MPU6050 accelgyro;
volatile bool accInterrupt = false;
unsigned long int stepcount = 0;
unsigned int detcdur = 5; //6
unsigned int thrs = 3;
unsigned int var;
boolean interruptA = false;

byte bcdToDec(byte value)
{
  return ((value / 16) * 10 + value % 16);
}

byte decToBcd(byte value) {
  return (value / 10 * 16 + value % 10);
}

void readPCF8563() {
  Wire.beginTransmission(PCF8563address);
  Wire.write(0x02);
  Wire.endTransmission();
  Wire.requestFrom(PCF8563address, 7);
  second     = bcdToDec(Wire.read() & B01111111); // remove VL error bit
  minute     = bcdToDec(Wire.read() & B01111111); // remove unwanted bits from MSB
  hour       = bcdToDec(Wire.read() & B00111111);
  dayOfMonth = bcdToDec(Wire.read() & B00111111);
  dayOfWeek  = bcdToDec(Wire.read() & B00000111);
  month      = bcdToDec(Wire.read() & B00011111);  // remove century bit, 1999 is over
  year       = bcdToDec(Wire.read());
}

void wakeUp() {
  while (!digitalRead(BUTTON)) {};
  BUTTON1++;
  if (BUTTON1 == 1) {
    u8g.sleepOff();
    delay(50);
  }
  else {
    if (BUTTON1 == 5) {
      BUTTON1 = 0;
      u8g.sleepOn();
    }
  }
  while (!digitalRead(BUTTON)) {};
  delay(100);
}

void wakeUpMotion() {
  if (interruptA == 0) {
    stepcount++;
    interruptA = true;
  }
}

void setup() {
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(ACCINTPIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  pinMode(0, OUTPUT);
  pinMode(1, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);

  digitalWrite(0, LOW);
  digitalWrite(1, LOW);
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);
  digitalWrite(8, LOW);
  digitalWrite(9, LOW);
  digitalWrite(10, LOW);
  digitalWrite(11, LOW);
  digitalWrite(12, LOW);
  digitalWrite(LED_PIN, LOW);

  startsensors();
  u8g.setFont(u8g_font_fub14r);
  attachInterrupt(0, wakeUp, LOW);
  attachInterrupt(1, wakeUpMotion, LOW);
  setBrightness();
  delay(100);
}

void setBrightness() {
  Wire.beginTransmission(0x3c);
  Wire.write(0x00);
  Wire.write(BRIGHTNESSREG);
  Wire.endTransmission();
  Wire.beginTransmission(0x3c);
  Wire.write(0x00);
  Wire.write(BRIGHTNESS);
  Wire.endTransmission();
}

void startsensors() {
  Wire.begin();
  delay(100);
  accelgyro.initialize();

  Wire.beginTransmission(0x68);
  Wire.write(0x37);
  Wire.write(0x02);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x6A);
  Wire.write(0x00);
  Wire.endTransmission();

  //Disable Sleep Mode
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  compass = HMC5883L();
  compass.SetScale();
  compass.SetMeasurementMode(Measurement_Idle);
  if (!bmp.begin(BMP085_ULTRALOWPOWER)) {
    while (1) {}
  }

  accelgyro.setIntMotionEnabled(1);//Set Motion Detection interrupt enabled status.
  accelgyro.setIntFreefallEnabled(1);
  accelgyro.setIntZeroMotionEnabled(0);
  accelgyro.setIntFIFOBufferOverflowEnabled(0);
  accelgyro.setIntDataReadyEnabled(0); //New interrupt enabled status
  accelgyro.setDHPFMode(3); //New high-pass filter configuration more than 1.25Hz pass
  accelgyro.setDLPFMode(5); //New low-pass filter configuration below 10Hz pass
  accelgyro.setMotionDetectionThreshold(thrs);  //20 - 2
  accelgyro.setMotionDetectionDuration(detcdur); //New motion detection duration threshold value (LSB = 1ms)
  accelgyro.setInterruptMode(1); //New interrupt mode (0=active-high, 1=active-low)
  accelgyro.setInterruptDrive(0); //New interrupt drive mode (0=push-pull, 1=open-drain)
  accelgyro.setInterruptLatch(0); //New latch mode (0=50us-pulse, 1=latch-until-int-cleared)
  accelgyro.setInterruptLatchClear(0); //New latch clear mode (0=status-read-only, 1=any-register-read)
  accelgyro.setRate(7); //Set the rate to disable the gyroscope
  accelgyro.setWakeFrequency(3); //Wake up the accelerometer at 1.25Hz to save power
  accelgyro.setWakeCycleEnabled(1); //Enable only accel. ON - Low power mode, waking it up from time to time

  /*          |   ACCELEROMETER    |           GYROSCOPE
  * DLPF_CFG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate
  * ---------+-----------+--------+-----------+--------+-------------
  * 0        | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8kHz
  * 1        | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz
  * 2        | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1kHz
  * 3        | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz
  * 4        | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz
  * 5        | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz
  * 6        | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz
  * ACCEL_HPF | Filter Mode | Cut-off Frequency
  * ----------+-------------+------------------
  * 0         | Reset       | None
  * 1         | On          | 5Hz
  * 2         | On          | 2.5Hz
  * 3         | On          | 1.25Hz
  * 4         | On          | 0.63Hz
  * 7         | Hold        | None
  * LP_WAKE_CTRL | Wake-up Frequency
  * -------------+------------------
  * 0            | 1.25 Hz
  * 1            | 2.5 Hz
  * 2            | 5 Hz
  * 3            | 10 H*/

  delay(50);
  accelgyro.setTempSensorEnabled(0);
  accelgyro.setStandbyXGyroEnabled(1);
  accelgyro.setStandbyYGyroEnabled(1);
  accelgyro.setStandbyZGyroEnabled(1);
  delay(50);
}

void loop() {
  while (BUTTON1 != 0) {
    setBrightness();
    u8g.firstPage();
    do {
      switch (BUTTON1) {
        case 0:
          break;
        case 1:
          drawtime();
          break;
        case 2:
          drawcompass();
          break;
        case 3:
          drawaltitude();
          break;
        case 4:
          drawsteps();
          break;
      }
    } while ( u8g.nextPage() );

    if ((millis() - timer_draw > 200) && (BUTTON1 == 2)) {
      compass.SetMeasurementMode(Measurement_Continuous);
      delay(10);
      compassreading();
      timer_draw = millis();
    }
  }

  //accelgyro.setSleepEnabled(1);
  compass.SetMeasurementMode(Measurement_Idle);
  delay(10);
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
  delay(10);
}

void drawaltitude(void) {
  char b[20];
  str1 = String((int)(bmp.readTemperature() - 2));
  str1 = str1 + "*C";
  str1.toCharArray(b, 11);
  u8g.drawStr(68, 18, b);
  str1 = String((long int)(bmp.readAltitude()));
  str1.toCharArray(b, 11);
  u8g.drawStr(68, 40, b);
  str1 = String((long int)(bmp.readPressure()));
  str1 = str1 + " m";
  str1.toCharArray(b, 11);
  u8g.drawStr(68, 60, b);
  u8g.drawLine(0, 0, 127, 0);
  u8g.drawLine(0, 22, 127, 22);
  u8g.drawLine(0, 42, 127, 42);
  u8g.drawLine(127, 0, 127, 63);
  u8g.drawLine(127, 63, 0, 63);
  u8g.drawLine(0, 63, 0, 0);
  u8g.drawStr( 2, 18, "Temp:");
  u8g.drawStr( 2, 40, "Altd:");
  u8g.drawStr( 2, 60, "Pres:");
}

void compassreading(void) {
  MagnetometerFinished finished = compass.ReadFinishedAxis();
  error = atan2(finished.YAxis, finished.XAxis);
  error = 180 - error * 180 / PI;
  if (error > 180) error = error - 360;
}

void drawcompass(void) {
  char b[20];
  str1 = String((int)error);
  str1 = str1 + "'";
  str1.toCharArray(b, 11);
  u8g.drawStr( 15, 56, b);
  if ((error > -23) && (error < 23)) str1 = "N";
  if ((error < -157) || (error > 157)) str1 = "S";
  if ((error > -113) && (error < -67)) str1 = "E";
  if ((error > 67) && (error < 113)) str1 = "W";
  if ((error >= -67) && (error <= -23)) str1 = "NE";
  if ((error >= 23) && (error <= 67)) str1 = "NW";
  if ((error >= -157) && (error <= -113)) str1 = "SE";
  if ((error >= 113) && (error <= 157)) str1 = "SW";
  str1.toCharArray(b, 11);
  u8g.drawStr( 24, 24, b);
  u8g.drawLine(0, 0, 127, 0);
  u8g.drawLine(0, 32, 75, 32);
  u8g.drawLine(127, 0, 127, 63);
  u8g.drawLine(127, 63, 0, 63);
  u8g.drawLine(0, 63, 0, 0);
  u8g.drawLine(75, 0, 75, 63);
  u8g.drawCircle(100, 32, 20);
  u8g.drawLine(100, 32, 100 + (20 * sin(error * PI / 180)), 32 - (20 * cos(error * PI / 180)));
}

void drawsteps(void) {
  char b[20];
  //unsigned long int number = 0;
  //EEPROMWritelong(0, 2220);
  //number = EEPROMReadlong(0);
  //str1 = String(number);
  //str1.toCharArray(b, 11);
  //u8g.drawStr(0, 40, b);
  str1 = String(stepcount);//passo);
  str1.toCharArray(b, 11);
  u8g.drawStr(65, 18, b);
  str1 = String((int)stepcount);//calorie);
  str1.toCharArray(b, 11);
  u8g.drawStr(65, 40, b);
  str1 = String((int)(((double)stepcount) * 0.75));
  str1.toCharArray(b, 11);
  u8g.drawStr(65, 60, b);
  u8g.drawLine(0, 0, 127, 0);
  u8g.drawLine(0, 22, 127, 22);
  u8g.drawLine(0, 42, 127, 42);
  u8g.drawLine(127, 0, 127, 63);
  u8g.drawLine(127, 63, 0, 63);
  u8g.drawLine(0, 63, 0, 0);
  u8g.drawStr( 2, 18, "Step:");
  u8g.drawStr( 2, 40, "Kcal:");
  u8g.drawStr( 2, 60, "Mts:");
}

void drawtime(void) {
  char b[20];
  readPCF8563();
  str1 = String(hour); //converting integer into a string
  if (hour < 10) str1 = "0" + str1;
  str1 = str1 + ":";
  if (minute < 10) str1 = str1 + "0" + String(minute);
  else str1 = str1 + String(minute);
  str1 = str1 + ":";
  if (second < 10) str1 = str1 + "0" + String(second);
  else str1 = str1 + String(second);
  str1.toCharArray(b, 11);
  u8g.drawStr( 24, 50, b);

  str1 = String(dayOfMonth);
  if (dayOfMonth < 10) str1 = "0" + str1;
  str1 = str1 + "/";
  if (month < 10) str1 = str1 + "0" + String(month);
  else str1 = str1 + String(month);
  str1 = str1 + "/" + "20" + String(year);
  str1.toCharArray(b, 11);
  u8g.drawStr( 11, 20, b);

  u8g.drawLine(0, 0, 127, 0);
  u8g.drawLine(0, 23, 127, 23);
  u8g.drawLine(127, 0, 127, 63);
  u8g.drawLine(127, 63, 0, 63);
  u8g.drawLine(0, 63, 0, 0);
}

void EEPROMWritelong(int address, long value) {
  //Decomposition from a long to 4 bytes by using bitshift.
  //One = Most significant -> Four = Least significant byte
  byte four = (value & 0xFF);
  byte three = ((value >> 8) & 0xFF);
  byte two = ((value >> 16) & 0xFF);
  byte one = ((value >> 24) & 0xFF);

  //Write the 4 bytes into the eeprom memory.
  EEPROM.write(address, four);
  EEPROM.write(address + 1, three);
  EEPROM.write(address + 2, two);
  EEPROM.write(address + 3, one);
}

long EEPROMReadlong(long address) {
  //Read the 4 bytes from the eeprom memory.
  long four = EEPROM.read(address);
  long three = EEPROM.read(address + 1);
  long two = EEPROM.read(address + 2);
  long one = EEPROM.read(address + 3);

  //Return the recomposed long by using bitshift.
  return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
}
