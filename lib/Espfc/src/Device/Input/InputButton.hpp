#pragma once

#include <cstdint>

namespace Espfc::Device::Input {

class InputButton
{
public:
  int begin(int pin, bool triggerLow = true);
  int update();

private:
  int _result = 0; // bit 0: one click, 1: double click, 2: long press
  int _pin = -1;
  bool _triggerLow = true;
};

}
