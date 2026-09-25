#include "Hal/Platform.hpp"
#include <Arduino.h>
#include <EEPROM.h>
#include <EscDriver.h>
#include <EspWire.h>
#include <Espfc.h>
#include <Gps.hpp>
#include <Hal/Platform.hpp>
#include <Kalman.hpp>
#include <Madgwick.hpp>
#include <Mahony.hpp>
#include <SPI.h>
#include <Wire.h>
#include <blackbox/blackbox.h>
#include <printf.h>
#if defined(ESPFC_HAL_ESPNOW)
#include <EspNowRcLink/Receiver.h>
#endif
#ifdef ESP32
void IRAM_ATTR serialEventRun(void) {}
#endif

Espfc::Espfc espfc;

#if ESPFC_HAL_CORE_COUNT > 1
#if defined(ESPFC_HAL_FREE_RTOS)

// ESP32 multicore
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

TaskHandle_t gyroTaskHandle = NULL;
TaskHandle_t pidTaskHandle = NULL;

bool IRAM_ATTR gyroTimerIsr(void* args)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(gyroTaskHandle, &xHigherPriorityTaskWoken);
  return xHigherPriorityTaskWoken == pdTRUE;
}

void gyroTask(void* pvParameters)
{
  espfc.begin();
  espfc.beginGyroTimer(gyroTimerIsr);
  while (true)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // wait for timer isr notification
    espfc.update(true);
  }
}

void pidTask(void* pvParameters)
{
  while (true)
  {
    espfc.updateOther();
  }
}

void setup()
{
  disableCore0WDT();
  // internal task priorities
  // PRO(0): hi-res timer(22), timer(1), event-loop(20), lwip(18/any), wifi(23), wpa(2/any), BT/vhci(23), NimBle(21),
  // BT/other(19,20,22), Eth(15), Mqtt(5/any) APP(1): free
  espfc.load();
  xTaskCreateUniversal(gyroTask, "gyroTask", 8192, NULL, 24, &gyroTaskHandle, 1);
  xTaskCreateUniversal(pidTask, "pidTask", 8192, NULL, 1, &pidTaskHandle, 0);
  vTaskDelete(NULL); // delete arduino loop task
}

void loop() {}

#elif defined(ESPFC_HAL_MULTI_CORE_RP2040)

bool core1_separate_stack = true;
volatile bool setup_done = false;

// RP2040 multicore
// TODO: https://emalliab.wordpress.com/2021/04/18/raspberry-pi-pico-arduino-core-and-timers/
void setup()
{
  espfc.load();
  espfc.begin();
  setup_done = true;
}

void loop()
{
  espfc.update();
}

void setup1()
{
  while (!setup_done)
    ;
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
}

void loop()
{
  espfc.update();
  espfc.updateOther();
}

#endif
