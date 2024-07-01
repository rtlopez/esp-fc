#if defined(ARCH_RP2040)

#include "EscDriverRP2040.h"
#include <hardware/gpio.h>
#include <hardware/pwm.h>
#include <hardware/dma.h>

// spec
//#define DSHOT150_T0H 2500u
//#define DSHOT150_T1H 5000u
//#define DSHOT150_T   6666u

// real
#define DSHOT150_T0H 2333u
#define DSHOT150_T1H 4666u
#define DSHOT150_T   6666u

int EscDriverRP2040::attach(size_t channel, int pin, int pulse)
{
  if(channel >= ESC_CHANNEL_COUNT) return 0;
  if(pin > 28) return 0;

  _slots[channel].pin = pin;
  _slots[channel].pulse = usToTicks(pulse);
  _slots[channel].slice = pwm_gpio_to_slice_num(pin);
  _slots[channel].channel = pwm_gpio_to_channel(pin);

  //pinMode(pin, OUTPUT);
  //digitalWrite(pin, LOW);

  pwm_config config = pwm_get_default_config();
  switch(_protocol)
  {
    case ESC_PROTOCOL_PWM:
    case ESC_PROTOCOL_ONESHOT125:
    case ESC_PROTOCOL_ONESHOT42:
    case ESC_PROTOCOL_MULTISHOT:
    case ESC_PROTOCOL_BRUSHED:
      pwm_config_set_clkdiv_int(&config, _divider);    /// Setting the divider to slow down the clock
      pwm_config_set_wrap(&config, _interval);         /// setting the Wrap time

      gpio_set_function(pin, GPIO_FUNC_PWM);           /// Set the pin to be PWM
      pwm_set_chan_level(_slots[channel].slice, _slots[channel].channel, _slots[channel].pulse);
      pwm_init(_slots[channel].slice, &config, true);  /// start PWM
      break;
    case ESC_PROTOCOL_DSHOT150:
    case ESC_PROTOCOL_DSHOT300:
    case ESC_PROTOCOL_DSHOT600:
      pwm_config_set_clkdiv_int(&config, 1);     /// Setting the divider to slow down the clock
      pwm_config_set_wrap(&config, _dt);         /// setting the Wrap time

      gpio_set_function(pin, GPIO_FUNC_PWM);           /// Set the pin to be PWM
      pwm_set_chan_level(_slots[channel].slice, _slots[channel].channel, 0);
      pwm_init(_slots[channel].slice, &config, true);  /// start PWM

      // DMA is allocated for slice on diferrent slot already.
      if (isSliceDriven(_slots[channel].slice)) return 1;

      // Setup DMA channel to drive the PWM
      _slots[channel].drive = true;
      _slots[channel].pwm_dma_chan = dma_claim_unused_channel(true);
      _slots[channel].dma_config = dma_channel_get_default_config(_slots[channel].pwm_dma_chan);

      // Transfers 32-bits at a time, increment read address so we pick up a new fade value each
      // time, don't increment writes address so we always transfer to the same PWM register.
      channel_config_set_transfer_data_size(&_slots[channel].dma_config, DMA_SIZE_32);
      channel_config_set_read_increment(&_slots[channel].dma_config, true);
      channel_config_set_write_increment(&_slots[channel].dma_config, false);

      // Transfer when PWM slice that is connected to the output asks for a new value
      //channel_config_set_dreq(&_slots[channel].dma_config, DREQ_PWM_WRAP0 + _slots[channel].slice);
      channel_config_set_dreq(&_slots[channel].dma_config, pwm_get_dreq(_slots[channel].slice));
      break;
    default:
      break;
  }
  return 1;
}

int EscDriverRP2040::pin(size_t channel) const
{
  if(channel < 0 || channel >= ESC_CHANNEL_COUNT) return -1;
  return _slots[channel].pin;
}

uint32_t EscDriverRP2040::telemetry(size_t channel) const
{
  return 0;
}

bool EscDriverRP2040::isSliceDriven(int slice)
{
  for(size_t i = 0; i < ESC_CHANNEL_COUNT; ++i) {
    if(_slots[i].active() && _slots[i].slice == slice && _slots[i].drive) return true;
  }
  return false;
}

int EscDriverRP2040::write(size_t channel, int pulse)
{
  if(channel >= ESC_CHANNEL_COUNT) return 0;
  _slots[channel].pulse = usToTicks(pulse);
  return 1;
}

void EscDriverRP2040::apply()
{
  if(_protocol >= ESC_PROTOCOL_DSHOT150 && _protocol <= ESC_PROTOCOL_DSHOT600)
  {
    dshotWriteDMA();
    return;
  }
  for(size_t i = 0; i < ESC_CHANNEL_COUNT; ++i)
  {
    if(!_slots[i].active()) continue;
    pwm_set_chan_level(_slots[i].slice, _slots[i].channel, _slots[i].pulse);
  }
}

uint32_t EscDriverRP2040::usToTicks(uint32_t us)
{
  uint32_t ticks = 0;
  switch(_protocol)
  {
    case ESC_PROTOCOL_ONESHOT125:
      ticks = map(us, 1000, 2000, usToTicksReal(125), usToTicksReal(250));
      break;
    case ESC_PROTOCOL_ONESHOT42:
      ticks = map(us, 1000, 2000, usToTicksReal(42), usToTicksReal(84));
      break;
    case ESC_PROTOCOL_MULTISHOT:
      ticks = map(us, 1000, 2000, usToTicksReal(5), usToTicksReal(25));
      break;
    case ESC_PROTOCOL_BRUSHED:
      ticks = map(constrain(us, 1000, 2000), 1000, 2000, 0, _interval);
      break;
    case ESC_PROTOCOL_DSHOT150:
    case ESC_PROTOCOL_DSHOT300:
    case ESC_PROTOCOL_DSHOT600:
      ticks = constrain(us, 1000, 2000);
      break;
    default:
      ticks = usToTicksReal(constrain(us, 800, 2200)); // PWM
      break;
  }
  return ticks;
}

uint32_t EscDriverRP2040::usToTicksReal(uint32_t us)
{
  uint64_t t = (uint64_t)us * (F_CPU / _divider);
  return t / 1000000ul;
}

uint32_t EscDriverRP2040::nsToDshotTicks(uint32_t ns)
{
  const uint32_t k = 128u;
  uint64_t d = 1000000000ull * k / F_CPU;
  uint32_t t = ns * k / d;
  return t;
}

EscDriverRP2040::EscDriverRP2040(): _protocol(ESC_PROTOCOL_PWM), _async(true), _rate(50), _timer(ESC_DRIVER_TIMER1), _divider(F_CPU / 3 / 1000000ul), _interval(usToTicksReal(1000000uL / _rate))
{
  for(size_t i = 0; i < ESC_CHANNEL_COUNT; ++i) _slots[i] = Slot();
}

int EscDriverRP2040::begin(const EscConfig& conf)
{
  _timer = (EscDriverTimer)conf.timer;
  _protocol = ESC_PROTOCOL_SANITIZE(conf.protocol);
  _async = _protocol == ESC_PROTOCOL_BRUSHED ? true : conf.async; // force async for brushed
  _rate = constrain(conf.rate, 50, 8000);
  _interval = usToTicksReal(1000000ul / _rate);

  _dl = nsToDshotTicks(DSHOT150_T0H);
  _dh = nsToDshotTicks(DSHOT150_T1H);
  _dt = nsToDshotTicks(DSHOT150_T);

  switch(_protocol)
  {
    // pulse delays experimentally selected
    case ESC_PROTOCOL_DSHOT150:
      break;
    case ESC_PROTOCOL_DSHOT300:
      _dl /= 2;
      _dh /= 2;
      _dt /= 2;
      break;
    case ESC_PROTOCOL_DSHOT600:
    case ESC_PROTOCOL_PROSHOT:
      _dl /= 4;
      _dh /= 4;
      _dt /= 4;
      break;
    case ESC_PROTOCOL_DISABLED:
      break;
    default: // analog
      break;
  }

  clearDmaBuffer();

  return 1;
}

void EscDriverRP2040::end()
{
  for(size_t i = 0; i < ESC_CHANNEL_COUNT; ++i)
  {
    if(!_slots[i].active()) continue;
    digitalWrite(_slots[i].pin, LOW);
  }
}

void EscDriverRP2040::clearDmaBuffer()
{
  for(size_t i = 0; i < NUM_PWM_SLICES; ++i) {
    for(size_t j = 0; j < DSHOT_BIT_COUNT + 1; j++) {
      _dma_buffer[i][j] = 0;
    }
  }
}

void EscDriverRP2040::dshotWriteDMA()
{
  clearDmaBuffer();

  // fill dma buffer
  for(size_t i = 0; i < ESC_CHANNEL_COUNT; ++i)
  {
    if(!_slots[i].active()) continue;

    uint16_t pulse = constrain(_slots[i].pulse, 0, 2000);
    uint16_t value = dshotConvert(pulse);
    uint16_t frame = dshotEncode(value);

    int slice = _slots[i].slice;
    int channel = _slots[i].channel;

    for(size_t j = 0; j < DSHOT_BIT_COUNT; j++)
    {
      const bool b = (frame >> (DSHOT_BIT_COUNT - 1 - j)) & 0x01;
      uint32_t v = (b ? _dh : _dl) & 0xffff;
      if(channel) {
        // channel B
        _dma_buffer[slice][j] |= v << 16;
      } else {
        // channel A
        _dma_buffer[slice][j] |= v;
      }
    }
  }

  // transfer DMA buffer to PWM
  for(size_t i = 0; i < ESC_CHANNEL_COUNT; ++i)
  {
    if(!_slots[i].active()) continue;
    if(!_slots[i].drive) continue;

    int slice = _slots[i].slice;

    // Trigger only once for slice
    dma_channel_configure(
      _slots[i].pwm_dma_chan,
      &_slots[i].dma_config,
      &pwm_hw->slice[slice].cc, // Write to PWM counter compare
      _dma_buffer[slice], // Read values from dshot buffer
      DSHOT_BIT_COUNT + 1, // 16+1 values to copy
      true // Start immediately.
    );
  }
}

#endif // ARCH_RP2040
