#pragma once

#include <cstddef>
#include <cstdint>

namespace Espfc::Hal {

// Preemptive task pinned to a cpu core, available on RTOS targets (see ESPFC_HAL_FREE_RTOS).
class Task
{
public:
  using Handle = void*;
  using Function = void (*)(void* arg);

  enum class Priority : uint8_t
  {
    Low,  // background work, yields to everything else
    High, // hard real time, gyro loop
  };

  // stackSize is in bytes, core is the cpu the task is pinned to
  static bool create(Function function, const char* name, size_t stackSize, void* arg, Priority priority, uint8_t core);

  // Handle of the calling task, to be used for notifications.
  static Handle currentHandle();

  // Wake a task from interrupt context, returns true when a context switch is required on ISR exit.
  static bool notifyFromIsr(Handle handle);

  // Block the calling task until notifyFromIsr() is called.
  static void waitNotify();

  // Terminate the calling task, does not return on RTOS targets.
  static void exitCurrent();

  // Busy looping tasks starve the idle task, which the ESP32 task watchdog monitors.
  static void disableIdleWatchdog();
};

} // namespace Espfc::Hal
