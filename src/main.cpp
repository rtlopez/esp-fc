#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>
#include <Espfc.h>
#include <I2Cdev.h>
#include <Adafruit_BMP280.h>
#include <MPU6050.h>
#include <HMC5883L.h>
#include <Kalman.h>
#include <MadgwickAHRS.h>
#include <printf.h>
#include <blackbox.h>

Espfc::Espfc espfc(Serial, Serial);

void setup()
{
  EEPROM.begin(1024);
  Wire.begin();
  //Wire.setClock(400000);
  Wire.setClock(1000000); // in real ~640kHz
  Serial.begin(115200);
  Serial.println();

  espfc.begin();
}

void loop()
{
  espfc.update();
  yield();
}

int main()
{
  setup();
  while(true) loop();
  return 0;
}
