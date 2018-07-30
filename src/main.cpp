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

#if defined(ESP32)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

Espfc::Espfc espfc;

#if defined(ESP32)
void otherTask(void *pvParameters)
{
  while(true) espfc.updateOther();
}
#endif

void setup()
{
  espfc.begin();
  #if defined(ESP32)
  xTaskCreatePinnedToCore(otherTask, "otherTask", 8192, NULL, 1, NULL, ARDUINO_RUNNING_CORE);
  #endif
}

void loop()
{
  espfc.update();
  #if !defined(ESP32)
  espfc.updateOther();
  #endif
}