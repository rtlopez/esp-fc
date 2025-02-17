#pragma once
#include <cstdint>
#include <cstddef>

namespace Espfc::Connect
{

enum LedStatus
{
  LED_OFF,
  LED_OK,
  LED_ERROR,
  LED_ON
};

class StatusLed
{

public:
  StatusLed();
  void begin(int8_t pin, uint8_t invert);
  void update();
  void setStatus(LedStatus newStatus, bool force = false);

private:
  void _write(uint8_t val);
  int8_t _pin;
  int8_t _invert;
  LedStatus _status;
  uint32_t _next;
  bool _state;
  size_t _step;
  int * _pattern;
};

}