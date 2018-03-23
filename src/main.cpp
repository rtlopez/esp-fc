#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>
#include <Espfc.h>
#include <Kalman.h>
#include <MadgwickAHRS.h>
#include <printf.h>
#include <blackbox/blackbox.h>
#include <EspSoftSerial.h>
#include <EspGpio.h>
#include <EscDriver.h>

Espfc::Espfc espfc;

void setup()
{
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
