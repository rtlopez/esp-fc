#pragma once

// https://www.raspberrypi.com/documentation/pico-sdk/high_level.html#queue
// https://techtutorialsx.com/2017/08/20/esp32-arduino-freertos-queues/
// https://www.freertos.org/a00116.html

namespace Espfc {

enum EventType
{
  EVENT_IDLE,
  EVENT_GYRO_READ,
  EVENT_ACCEL_READ,
  EVENT_DISARM,
};

class Event
{
  public:
    Event(): type(EVENT_IDLE) {}
    Event(EventType t): type(t) {}
    Event(const Event& e) = default;
  public:
    EventType type;
};

}

#if defined(ARCH_RP2040)
  #include <pico/util/queue.h>
  using TargetQueueHandle = queue_t;
#elif defined(ESPFC_FREE_RTOS_QUEUE)
  #include <freertos/queue.h>
  using TargetQueueHandle = QueueHandle_t;
#elif defined(ESPFC_ATOMIC_QUEUE)
  #include "QueueAtomic.h"
  using TargetQueueHandle = Espfc::QueueAtomic<Espfc::Event, 63>;
#elif defined(UNIT_TEST) || !defined(ESPFC_MULTI_CORE)
  using TargetQueueHandle = int;
#else
  #error "Not yet implelented multicore queue"
#endif

namespace Espfc {

namespace Target {

class Queue
{
  public:
    void begin();
    void send(const Event& e);
    Event receive();
    bool isEmpty() const;
    bool isFull() const;

  private:
    TargetQueueHandle _q;
};

}

}
