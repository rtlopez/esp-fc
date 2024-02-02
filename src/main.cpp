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

#ifdef ESP32
void serialEventRun(void) {}
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
      espfc.beginOther();
      xTaskNotifyGive(loopTaskHandle);
      while(true)
      {
        espfc.updateOther();
      }
    }
    void setup()
    {
      espfc.load();
      espfc.begin();
      disableCore0WDT();
      xTaskCreateUniversal(otherTask, "otherTask", 8192, NULL, 1, &otherTaskHandle, 0); // run on PRO(0) core, loopTask is on APP(1)
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // wait for `otherTask` initialization
    }
    void loop()
    {
      //const uint32_t timeout = millis() + 200;
      //while(millis() < timeout)
      //{
        espfc.update();
      //}
    }

  #elif defined(ESPFC_MULTI_CORE_RP2040)

    // RP2040 multicore
    volatile static bool setup1Done = false;
    void setup1()
    {
      espfc.beginOther();
      setup1Done = true;
    }
    void setup()
    {
      espfc.load();
      espfc.begin();
      while(!setup1Done); //wait for setup1()
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
    espfc.begin();
    espfc.beginOther();
  }
  void loop()
  {
    espfc.update();
    espfc.updateOther();
  }

#endif
