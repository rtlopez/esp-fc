#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>
#include <Espfc.h>
#include <Kalman.h>
#include <Madgwick.h>
#include <Mahony.h>
#include <printf.h>
#include <blackbox/blackbox.h>
#include <EspSoftSerial.h>
#include <EspGpio.h>
#include <EscDriver.h>
#include <EspWire.h>
#include "Debug.h"

#ifdef ESPFC_FREE_RTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
  #ifdef ESPFC_MULTI_CORE
    TaskHandle_t otherTaskHandle = NULL;
    extern TaskHandle_t loopTaskHandle;
  #endif
#endif

#ifdef ESPFC_WIFI_ALT
  #include <ESP8266WiFi.h>
#else
  #include <WiFi.h>
#endif

Espfc::Espfc espfc;

#ifdef ESPFC_MULTI_CORE
void otherTask(void *pvParameters)
{
  espfc.beginOther();
#ifdef ESPFC_FREE_RTOS
  xTaskNotifyGive(loopTaskHandle);
#endif
  while(true)
  {
    espfc.updateOther();
  }
}
#endif

void setup()
{
  Serial.begin(115200);
  const uint32_t timeout = millis() + 2000;
  while (!Serial) {
    if(millis() > timeout) break;
  };
  Espfc::initDebugStream(&Serial);
  delay(1000);

  espfc.load();
#ifdef ESPFC_MULTI_CORE
#ifdef ESPFC_FREE_RTOS
  disableCore0WDT();
  xTaskCreatePinnedToCore(otherTask, "otherTask", 8192, NULL, 1, &otherTaskHandle, 0); // run on PRO(0) core, loopTask is on APP(1)
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // wait for `otherTask` initialization
#endif
  espfc.begin();
#else
  espfc.beginOther();
  espfc.begin();
#endif
}

void loop()
{
  espfc.update();
#ifndef ESPFC_MULTI_CORE
  espfc.updateOther();
#endif
}