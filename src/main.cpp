#include "Hal/Platform.hpp"
#include <Arduino.h>
#include <EEPROM.h>
#include <EscDriver.h>
#include <EspWire.h>
#include <Espfc.h>
#include <Gps.hpp>
#include <Hal/FastCode.hpp>
#include <Hal/Platform.hpp>
#include <Hal/Task.hpp>
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

// RTOS multicore, gyro loop is triggered by the hardware timer interrupt
using Espfc::Hal::Task;

Task::Handle gyroTaskHandle = nullptr;

bool ISR_CODE_ATTR gyroTimerIsr(void* args)
{
  return Task::notifyFromIsr(gyroTaskHandle);
}

void gyroTask(void* pvParameters)
{
  gyroTaskHandle = Task::currentHandle();
  espfc.begin();
  espfc.beginGyroTimer(gyroTimerIsr);
  while (true)
  {
    Task::waitNotify(); // wait for timer isr notification
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
  Task::disableIdleWatchdog();
  espfc.load();
  Task::create(gyroTask, "gyroTask", 8192, nullptr, Task::Priority::High, 1);
  Task::create(pidTask, "pidTask", 8192, nullptr, Task::Priority::Low, 0);
  Task::exitCurrent(); // delete arduino loop task
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
