#pragma once

#include <cstddef>
#include <cstdint>

#include "Device/InputDevice.h"

namespace Espfc {
namespace Device {

/**
 * Parallel PWM receiver input.
 *
 * One GPIO per RC channel.
 * Normal RC PWM pulse width: approximately 1000–2000 us, repeated around 50 Hz.
 *
 * Channel order follows esp-fc raw receiver order:
 *   0 roll
 *   1 pitch
 *   2 throttle
 *   3 yaw
 *   4 aux1
 *   5 aux2
 *   6 aux3
 *   7 aux4
 */
class InputPWM : public InputDevice
{
public:
  static constexpr size_t CHANNELS = 8;

  InputPWM();

  /**
   * Configure PWM input pins.
   *
   * pins[i] == -1 disables that channel.
   * At least one pin should be configured by the caller.
   */
  void begin(const int8_t* pins, size_t len);

  InputStatus update() override;
  uint16_t get(uint8_t channel) const override;
  void get(uint16_t* data, size_t len) const override;
  size_t getChannelCount() const override;
  bool needAverage() const override;

private:
  static constexpr uint16_t MID_US = 1500;
  static constexpr uint16_t THROTTLE_LOW_US = 1000;

  static constexpr uint32_t PULSE_MIN_US = 700;
  static constexpr uint32_t PULSE_MAX_US = 2300;
  static constexpr uint32_t SIGNAL_LOST_US = 300000UL;

  struct Channel
  {
    int8_t pin = -1;

    volatile uint32_t riseUs = 0;
    volatile uint32_t lastUs = 0;
    volatile uint16_t widthUs = MID_US;
    volatile bool updated = false;
  };

  struct IsrArg
  {
    InputPWM* self = nullptr;
    uint8_t index = 0;
  };

  Channel _channels[CHANNELS];
  IsrArg _args[CHANNELS];

  static uint16_t defaultValue(size_t channel);

  void handle(uint8_t channel);
  static void handleIsr(void* arg);
};

} // namespace Device
} // namespace Espfc