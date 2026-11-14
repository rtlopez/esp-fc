#pragma once

#if defined(ARCH_RP2040)

#include "Hal/EscDriver.hpp"
#include <cstddef>
#include <cstdint>
#include <hardware/dma.h>
#include <hardware/pwm.h>

// TODO:
// https://cocode.se/linux/raspberry/pwm.html
// https://forums.raspberrypi.com/viewtopic.php?t=305969 (irq)
// https://github.com/raspberrypi/pico-examples/blob/master/pwm/led_fade/pwm_led_fade.c (irq)

namespace Hal::Detail {

class EscDriverImpl
{
public:
  class Slot
  {
  public:
    Slot(): pin(-1), pulse(0), slice(0), channel(0), drive(false) {}
    int pin;
    int pulse;
    int slice;
    int channel;
    bool drive;
    int pwm_dma_chan;
    dma_channel_config dma_config;
    inline bool active() const
    {
      return pin != -1;
    }
  };

  using mask_t = uint32_t;

  EscDriverImpl();

  bool begin(const EscConfig& conf);
  void end();
  bool attach(size_t channel, int pin, int pulse);
  void write(size_t channel, int pulse);
  int pin(size_t channel) const;
  uint32_t telemetry(size_t channel) const;
  void apply();

private:
  uint32_t usToTicks(uint32_t us);
  uint32_t usToTicksReal(uint32_t us);
  uint32_t nsToDshotTicks(uint32_t ns);
  void dshotWriteDMA();
  bool isSliceDriven(int slice);
  void clearDmaBuffer();

  EscProtocol _protocol;
  bool _async;
  int _rate;
  uint16_t _divider;
  uint32_t _interval;

  Slot _slots[EscDriver::CHANNEL_COUNT];

  int _dh;
  int _dl;
  int _dt;

  uint32_t _dma_buffer[NUM_PWM_SLICES][EscDriver::DSHOT_BIT_COUNT + 1];
};

} // namespace Hal::Detail

#endif // ARCH_RP2040
