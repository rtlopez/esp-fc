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
#include <WiFi.h>
#if !CONFIG_FREERTOS_UNICORE
#define ESPFC_DUAL_CORE 1
#endif
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#endif

Espfc::Espfc espfc;

#if defined(ESPFC_DUAL_CORE)
void otherTask(void *pvParameters)
{
  while(true) espfc.updateOther();
}
#endif

void setup()
{
  espfc.begin();
  #if defined(ESPFC_DUAL_CORE)
  xTaskCreatePinnedToCore(otherTask, "otherTask", 8192, NULL, 1, NULL, 0); // run on PRO(0) core, loopTask is on APP(1)
  #endif
}

void loop()
{
  espfc.update();
  #if !defined(ESPFC_DUAL_CORE)
  espfc.updateOther();
  #endif
}