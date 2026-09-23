#pragma once

#include "Hal/Queue.hpp"

namespace Espfc {

enum EventType
{
  EVENT_IDLE,
  EVENT_GYRO_READ,
  EVENT_ACCEL_READ,
  EVENT_DISARM,
};

struct Event
{
  Event(): type(EVENT_IDLE) {}
  Event(EventType t): type(t) {}
  Event(const Event& e) = default;
  EventType type;
};

using EventQueue = Hal::Queue<Event, 16>;

} // namespace Espfc
