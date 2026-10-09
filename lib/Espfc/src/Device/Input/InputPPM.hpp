#pragma once

#include "Device/InputDevice.hpp"
#include "Hal/Gpio.hpp"
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace Espfc {

enum PPMInvert
{
  PPM_MODE_NORMAL = 0,  // RISING edge
  PPM_MODE_INVERTED = 1 // FALLING edge
};

namespace Device::Input {

class InputPPM : public InputDevice
{
public:
  void begin(int8_t pin, PPMInvert invert = PPM_MODE_NORMAL);
  InputStatus update() override;
  uint16_t get(uint8_t i) const override;
  void get(uint16_t* data, size_t len) const override;
  size_t getChannelCount() const override;
  bool needAverage() const override;

private:
  static constexpr size_t CHANNELS = 16;

  void handle();
  static void handle_isr(void* args);

  std::atomic<int> _channels[CHANNELS]{};
  std::atomic<size_t> _write_count{0};
  size_t _read_count = 0;
  uint32_t _last_tick = 0;
  uint8_t _channel = 0;
  int8_t _pin = -1;
};

} // namespace Device::Input

} // namespace Espfc
