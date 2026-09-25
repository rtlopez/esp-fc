#if defined(ARCH_RP2040)

#include "Hal/Task.hpp"
#include <FreeRTOS.h>
#include <task.h>

namespace Espfc::Hal {

// the arduino core reserves the top priorities: idle other core helpers (MAX - 1), lwip and usb (MAX - 2),
// setup()/loop() and setup1()/loop1() run at MAX / 2
static inline UBaseType_t taskPriority(Task::Priority priority)
{
  return priority == Task::Priority::High ? configMAX_PRIORITIES - 1 : 1;
}

bool Task::create(Function function, const char* name, size_t stackSize, void* arg, Priority priority, uint8_t core)
{
  TaskHandle_t task = nullptr;
  // FreeRTOS takes the stack depth in words
  const configSTACK_DEPTH_TYPE depth = stackSize / sizeof(StackType_t);
  if (xTaskCreate(function, name, depth, arg, taskPriority(priority), &task) != pdPASS) return false;

  vTaskCoreAffinitySet(task, 1u << core);
  return true;
}

Task::Handle Task::currentHandle()
{
  return xTaskGetCurrentTaskHandle();
}

bool __not_in_flash_func(Task::notifyFromIsr)(Handle handle)
{
  BaseType_t woken = pdFALSE;
  vTaskNotifyGiveFromISR(static_cast<TaskHandle_t>(handle), &woken);
  // the hardware alarm handler ignores the result, request the context switch here
  portYIELD_FROM_ISR(woken);
  return false;
}

void __not_in_flash_func(Task::waitNotify)()
{
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
}

void Task::exitCurrent()
{
  vTaskDelete(nullptr);
}

void Task::disableIdleWatchdog() {}

} // namespace Espfc::Hal

#endif
