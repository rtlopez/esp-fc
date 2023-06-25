#pragma once

// https://www.raspberrypi.com/documentation/pico-sdk/high_level.html#queue
// https://techtutorialsx.com/2017/08/20/esp32-arduino-freertos-queues/
// https://www.freertos.org/a00116.html

#if defined(ARCH_RP2040)
  #include <pico/util/queue.h>
  typedef queue_t TargetQueueHandle;
#elif defined(ESPFC_FREE_RTOS)
  #include <freertos/queue.h>
  typedef QueueHandle_t TargetQueueHandle;
#elif
  #error "Not yet implelented multicore queue"
#endif

namespace Espfc {

enum EventType
{
  EVENT_IDLE,
  EVENT_START,
  EVENT_GYRO_READ,
  EVENT_GYRO_FILTER,
  EVENT_ACCEL_READ,
  EVENT_ACCEL_FILTER,
  EVENT_MAG_READ,
  EVENT_MAG_FILTER,
  EVENT_BARO_READ,
  EVENT_BARO_FILTER,
  EVENT_INPUT_READ,
  EVENT_MODE_UPDATE,
  EVENT_VOLTAGE_READ,
  EVENT_SERIAL_READ,
  EVENT_IMU_UPDATE,
  EVENT_PID_UPDATE,
  EVENT_MIXER_UPDATE,
  EVENT_BBLOG_UPDATE,
};

class Event
{
  public:
    Event(): type(EVENT_IDLE) {}
    Event(EventType t): type(t) {}
    Event(const Event& e): type(e.type) {}

  public:
    const EventType type;
};

namespace Target {

class Queue
{
  public:
    void begin();
    void send(const Event& e);
    Event reveive();
    bool isEmpty() const;

  private:
    TargetQueueHandle _q;
};

}

}
