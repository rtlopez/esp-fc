#if defined(ARCH_RP2040)

#include "EscDriverRP2040.h"
#include <hardware/gpio.h>
#include <hardware/pwm.h>
#include <hardware/dma.h>

int EscDriverRP2040::attach(size_t channel, int pin, int pulse)
{
  if(channel >= ESC_CHANNEL_COUNT) return 0;
  if(pin > 28) return 0;
  _slots[channel].pin = pin;
  _slots[channel].pulse = usToTicks(pulse);
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);

  if(_protocol <= ESC_PROTOCOL_BRUSHED)
  {
    _slots[channel].slice = pwm_gpio_to_slice_num(pin);
    _slots[channel].channel = pwm_gpio_to_channel(pin);

    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv_int(&config, _divider);    /// Setting the divider to slow down the clock
    pwm_config_set_wrap(&config, _interval);         /// setting the Wrap time

    gpio_set_function(pin, GPIO_FUNC_PWM);           /// Set the pin to be PWM
    pwm_init(_slots[channel].slice, &config, true);  /// start PWM
  }
  else if(_protocol >= ESC_PROTOCOL_DSHOT150 && _protocol <= ESC_PROTOCOL_DSHOT600)
  {
    _slots[channel].slice = pwm_gpio_to_slice_num(pin);
    _slots[channel].channel = pwm_gpio_to_channel(pin);

    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv_int(&config, 1);     /// Setting the divider to slow down the clock
    pwm_config_set_wrap(&config, _dt);         /// setting the Wrap time

    gpio_set_function(pin, GPIO_FUNC_PWM);           /// Set the pin to be PWM
    pwm_init(_slots[channel].slice, &config, true);  /// start PWM

    // DMA is allocated for slice on diferrent slot.
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
    channel_config_set_dreq(&_slots[channel].dma_config, DREQ_PWM_WRAP0 + _slots[channel].slice);
  }
  return 1;
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
    //case ESC_PROTOCOL_DSHOT1200:
      ticks = us;
      break;
    default:
      ticks = usToTicksReal(us); // PWM
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

int EscDriverRP2040::begin(EscProtocol protocol, bool async, int16_t rate, EscDriverTimer timer)
{
  _timer = timer;
  _protocol = ESC_PROTOCOL_SANITIZE(protocol);
  _async = _protocol == ESC_PROTOCOL_BRUSHED ? true : async; // force async for brushed
  _rate = constrain(rate, 50, 8000);
  _interval = usToTicksReal(1000000ul / _rate);

  switch(_protocol)
  {
    // pulse delays experimentally selected
    case ESC_PROTOCOL_DSHOT150:
      //{
      //  const float base = 140;
      //  _dh = lrintf(base * 0.35f);
      //  _dl = lrintf(base * 0.30f);
      //}
      _dl = nsToDshotTicks(2336u) / 4;
      _dh = nsToDshotTicks(4680u) / 4;
      _dt = nsToDshotTicks(6680u) / 4;
      break;
    case ESC_PROTOCOL_DSHOT300:
      //{
      //  const float base = 68;
      //  _dh = lrintf(base * 0.35f);
      //  _dl = lrintf(base * 0.30f);
      //}
      _dl = nsToDshotTicks(2336u) / 2;
      _dh = nsToDshotTicks(4680u) / 2;
      _dt = nsToDshotTicks(6680u) / 2;
      break;
    case ESC_PROTOCOL_DSHOT600:
    case ESC_PROTOCOL_PROSHOT:
      //{
      //  const float base = 31;
      //  _dh = lrintf(base * 0.36f);
      //  _dl = lrintf(base * 0.28f);
      //}
      _dl = nsToDshotTicks(2336u);
      _dh = nsToDshotTicks(4680u);
      _dt = nsToDshotTicks(6680u);
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
    for(size_t j = 0; j < DSHOT_BIT_COUNT; j++) {
      _dma_buffer[i][j] = 0;
    }
  }
}

uint32_t EscDriverRP2040::dshotWriteDMA()
{
  // fill dma buffer
  for(size_t i = 0; i < ESC_CHANNEL_COUNT; ++i)
  {
    if(!_slots[i].active()) continue;

    int pulse = constrain(_slots[i].pulse, 0, 2000);
    int value = 0; // disarmed
    // scale to dshot commands (0 or 48-2047)
    if(pulse > 1000)
    {
      value =  PWM_TO_DSHOT(pulse);
    }
    uint16_t frame = dshotEncode(value);

    int slice = _slots[i].slice;
    int channel = _slots[i].channel;

    for(size_t j = 0; j < DSHOT_BIT_COUNT; j++)
    {
      const bool b = (frame >> (DSHOT_BIT_COUNT - 1 - j)) & 0x01;
      uint32_t v = b ? _dh : _dl;
      if(channel) v <<= 16;
      _dma_buffer[slice][j] |= v;
    }
  }

  // push buffer to PWM over DMA
  for(size_t i = 0; i < ESC_CHANNEL_COUNT; ++i)
  {
    if(!_slots[i].active()) continue;
    int slice = _slots[i].slice;

    if (!isSliceDriven(slice)) continue;

    // Trigger only once for slice
    dma_channel_configure(
      _slots[i].pwm_dma_chan,
      &_slots[i].dma_config,
      &pwm_hw->slice[slice].cc, // Write to PWM counter compare
      _dma_buffer[slice], // Read values from fade buffer
      DSHOT_BIT_COUNT, // 16 values to copy
      true // Start immediately.
    );
  }

  return 0;
}

static inline void dshotDelay(int delay)
{
  while(delay--) __asm__ __volatile__ ("nop");
}

uint32_t EscDriverRP2040::dshotWriteBB()
{
  mask_t pinMask[DSHOT_BIT_COUNT];
  mask_t clrMask[DSHOT_BIT_COUNT];

  // zero mask arrays
  for(size_t i = 0; i < DSHOT_BIT_COUNT; i++)
  {
    pinMask[i] = 0;
    clrMask[i] = 0;
  }

  // compute bit masks
  for(size_t c = 0; c < ESC_CHANNEL_COUNT; c++)
  {
    if(!_slots[c].active()) continue;
    int pulse = constrain(_slots[c].pulse, 0, 2000);
    int value = 0; // disarmed
    // scale to dshot commands (0 or 48-2047)
    if(pulse > 1000)
    {
      value =  PWM_TO_DSHOT(pulse);
    }
    uint16_t frame = dshotEncode(value);
    mask_t mask = (1U << _slots[c].pin);
    for(size_t i = 0; i < DSHOT_BIT_COUNT; i++)
    {
      int val = (frame >> (DSHOT_BIT_COUNT - 1 - i)) & 0x01;
      pinMask[i] |= mask; // set leading bit and clear trailing bit on pin
      clrMask[i] |= (val ? 0ul : mask); // middle data bit on pin
    }
  }

  // worm-up io registers to avoid timing jitter
  sio_hw->gpio_set = 0ul;
  sio_hw->gpio_clr = 0ul;

  mask_t * pm = pinMask;
  mask_t * cm = clrMask;

  // write output
  for(size_t i = 0; i < DSHOT_BIT_COUNT; i++)
  {
    gpio_set_mask(*pm); dshotDelay(_dh);
    gpio_clr_mask(*cm); dshotDelay(_dh);
    gpio_clr_mask(*pm); dshotDelay(_dl);
    pm++;
    cm++;
  }

  return 0;
}

#endif // ARCH_RP2040
