#ifdef ESP32
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

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
#include <EspWire.h>

Espfc::Espfc espfc;

#ifdef ESP32
void otherTask(void *pvParameters)
{
  espfc.beginOther();
  while(true) espfc.updateOther();
}
#endif

void setup()
{
  espfc.begin();
  #ifdef ESP32
  xTaskCreatePinnedToCore(otherTask, "wifiTask", 8192, NULL, 1, NULL, 0);
  #endif
}

void loop()
{
  espfc.update();
  //yield();
}

/*int main()
{
  setup();
  while(true) loop();
  return 0;
}*/
