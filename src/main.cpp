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

#if defined(ESP32)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
//#include "esp_task_wdt.h"
#include <WiFi.h>
  #if !CONFIG_FREERTOS_UNICORE
    #define ESPFC_DUAL_CORE 1
    TaskHandle_t otherTaskHandle = NULL;
    extern TaskHandle_t loopTaskHandle;
  #endif
#elif defined(ESP8266)
  #include <ESP8266WiFi.h>
#endif

Espfc::Espfc espfc;

#if defined(ESPFC_DUAL_CORE)
void otherTask(void *pvParameters)
{
  espfc.beginOther();
  xTaskNotifyGive(loopTaskHandle);
  while(true)
  {
    espfc.updateOther();
  }
}
#endif

void setup()
{
  espfc.load();
#if defined(ESPFC_DUAL_CORE)
  disableCore0WDT();
  xTaskCreatePinnedToCore(otherTask, "otherTask", 8192, NULL, 1, &otherTaskHandle, 0); // run on PRO(0) core, loopTask is on APP(1)
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // wait for `otherTask` initialization
  espfc.begin();
#else
  espfc.beginOther();
  espfc.begin();
#endif
}

void loop()
{
  espfc.update();
  #if !defined(ESPFC_DUAL_CORE)
  espfc.updateOther();
  #endif
}