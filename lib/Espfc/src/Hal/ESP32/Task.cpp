#if defined(ESP32)

#include "Hal/Task.hpp"
#include "Hal/FastCode.hpp"
#include <esp32-hal.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace Espfc::Hal {

// internal task priorities
// PRO(0): hi-res timer(22), timer(1), event-loop(20), lwip(18/any), wifi(23), wpa(2/any), BT/vhci(23), NimBle(21),
// BT/other(19,20,22), Eth(15), Mqtt(5/any)
// APP(1): free
static inline UBaseType_t taskPriority(Task::Priority priority)
{
  return priority == Task::Priority::High ? 24 : 1;
}

bool Task::create(Function function, const char* name, size_t stackSize, void* arg, Priority priority, uint8_t core)
{
  // xTaskCreateUniversal takes the stack size in bytes and pins the task on multi core targets
  return xTaskCreateUniversal(function, name, stackSize, arg, taskPriority(priority), nullptr, core) == pdPASS;
}

Task::Handle Task::currentHandle()
{
  return xTaskGetCurrentTaskHandle();
}

bool ISR_CODE_ATTR Task::notifyFromIsr(Handle handle)
{
  BaseType_t woken = pdFALSE;
  vTaskNotifyGiveFromISR(static_cast<TaskHandle_t>(handle), &woken);
  return woken == pdTRUE;
}

void FAST_CODE_ATTR Task::waitNotify()
{
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
}

void Task::exitCurrent()
{
  vTaskDelete(nullptr);
}

void Task::disableIdleWatchdog()
{
  disableCore0WDT();
}

} // namespace Espfc::Hal

#endif
