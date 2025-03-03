#include "Device/Input/InputButton.hpp"
#include "Hal/Gpio.h"
#include <Arduino.h>
#include <MultiButton.h>

// https://github.com/poelstra/arduino-multi-button
namespace {

const static MultiButtonConfig conf = { 20, 250, 300 };
static MultiButton btn(&conf);

}

namespace Espfc::Device::Input {

int InputButton::begin(int pin, bool triggerLow)
{
  _pin = pin;
  _triggerLow = triggerLow;

  if(_pin == -1) return 0;

  Hal::Gpio::pinMode(_pin, INPUT_PULLUP);

  return 1;
}

int InputButton::update()
{
  if(_pin == -1) return 0;

  bool pressed = Hal::Gpio::digitalRead(_pin) ^ _triggerLow;

  btn.update(pressed);

  if (btn.isSingleClick())
  {
    _result ^= 1 << 0; // toggle bit 0
  }

  if (btn.isDoubleClick())
  {
    _result ^= 1 << 1; // toggle bit 1
  }

  if (btn.isLongClick())
  {
    _result ^= 1 << 2; // toggle bit 2
  }

  return _result;
}

}
