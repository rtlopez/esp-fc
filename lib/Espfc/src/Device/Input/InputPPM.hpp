#pragma once

#include "Device/InputDevice.hpp"
#include "Hal/Gpio.hpp"
#include <cstddef>
#include <cstdint>

namespace Espfc {

enum PPMMode
{
  PPM_MODE_NORMAL = Hal::Gpio::Rising,   // RISING edge
  PPM_MODE_INVERTED = Hal::Gpio::Falling // FALLING edge
};

namespace Device::Input {

class InputPPM : public InputDevice
{
public:
  void begin(int8_t pin, int mode = PPM_MODE_NORMAL);
  InputStatus update() override;
  uint16_t get(uint8_t i) const override;
  void get(uint16_t* data, size_t len) const override;
  size_t getChannelCount() const override;
  bool needAverage() const override;

private:
  static constexpr size_t CHANNELS = 16;

  void handle();
  static void handle_isr(void* args);

  volatile uint16_t _channels[CHANNELS] = {0};
  volatile uint32_t _last_tick = 0;
  volatile uint8_t _channel = 0;
  volatile bool _new_data = false;
  int8_t _pin = -1;
};

} // namespace Device::Input

} // namespace Espfc
