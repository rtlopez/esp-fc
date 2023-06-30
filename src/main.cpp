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
#include "Debug_Espfc.h"

#ifdef ESPFC_WIFI_ALT
  #include <ESP8266WiFi.h>
#else
  #include <WiFi.h>
#endif

Espfc::Espfc espfc;

#ifdef ESPFC_MULTI_CORE
  #if defined(ESPFC_FREE_RTOS)

    // ESP32 multicore
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    TaskHandle_t otherTaskHandle = NULL;
    extern TaskHandle_t loopTaskHandle;

    void otherTask(void *pvParameters)
    {
      espfc.load();
      espfc.beginOther();
      xTaskNotifyGive(loopTaskHandle);
      while(true)
      {
        espfc.updateOther();
      }
    }
    void setup()
    {
      disableCore0WDT();
      //xTaskCreatePinnedToCore(otherTask, "otherTask", 8192, NULL, 1, &otherTaskHandle, 0); // run on PRO(0) core, loopTask is on APP(1)
      xTaskCreateUniversal(otherTask, "otherTask", 8192, NULL, 1, &otherTaskHandle, 0); // run on PRO(0) core, loopTask is on APP(1)
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // wait for `otherTask` initialization
      espfc.begin();
    }
    void loop()
    {
      espfc.update();
    }

  #elif defined(ESPFC_MULTI_CORE_RP2040)

    // RP2040 multicore
    volatile static bool setup1Done = false;
    void setup1()
    {
      espfc.load();
      espfc.beginOther();
      setup1Done = true;
    }
    void setup()
    {
      while(!setup1Done); //wait for setup1()
      espfc.begin();
    }
    void loop()
    {
      espfc.update();
    }
    void loop1()
    {
      espfc.updateOther();
    }

  #else
    #error "No RTOS defined for multicore board"
  #endif

#else

  // single core
  void setup()
  {
    espfc.load();
    espfc.beginOther();
    espfc.begin();
  }
  void loop()
  {
    espfc.update();
    espfc.updateOther();
  }

#endif
