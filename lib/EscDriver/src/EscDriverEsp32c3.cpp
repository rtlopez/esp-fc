#if defined(ESP32C3)

#include "EscDriverEsp32c3.h"
#include <driver/timer.h>
#include <algorithm>

#define ESC_TIMER_DIVIDER 2
#define ESC_TIMER_IDX TIMER_0

#define DELTA_TICKS_MAX ((APB_CLK_FREQ / ESC_TIMER_DIVIDER / 1000000L) * 50000L)
#define DELTA_TICKS_MIN 5

#define TIMER_WAIT_SHORT_COMP 10UL
#define TIMER_WAIT_EDGE 240UL
#define TIMER_WAIT_COMP 90UL

void EscDriverEsp32c3::_isr_init(EscDriverTimer timer, void * driver)
{
  timer_group_t group = (timer_group_t)timer;
  timer_config_t config = {
      .alarm_en = TIMER_ALARM_EN,
      .counter_en = TIMER_PAUSE,
      .intr_type = TIMER_INTR_LEVEL,
      .counter_dir = TIMER_COUNT_UP,
      .auto_reload = TIMER_AUTORELOAD_DIS,
      .divider = ESC_TIMER_DIVIDER,
  };
  timer_init(group, ESC_TIMER_IDX, &config);
  timer_set_counter_value(group, ESC_TIMER_IDX, 0);
  timer_isr_callback_add(group, ESC_TIMER_IDX, EscDriverEsp32c3::handle, driver, ESP_INTR_FLAG_IRAM);
  timer_enable_intr(group, ESC_TIMER_IDX);
  timer_start(group, ESC_TIMER_IDX);
}

bool EscDriverEsp32c3::_isr_wait(EscDriverTimer timer, const uint32_t ticks)
{
  if(ticks >= TIMER_WAIT_EDGE) { // yield
    uint64_t value = timer_group_get_counter_value_in_isr((timer_group_t)timer, ESC_TIMER_IDX);
    value += (ticks - TIMER_WAIT_COMP);
    timer_group_set_alarm_value_in_isr((timer_group_t)timer, ESC_TIMER_IDX, value);
    timer_group_enable_alarm_in_isr((timer_group_t)timer, ESC_TIMER_IDX);
    return true;
  }

  if(ticks > 20) { // busy delay
    const uint32_t end = ESP.getCycleCount() + (2 * ticks * ESC_TIMER_DIVIDER) - TIMER_WAIT_SHORT_COMP;
    while(ESP.getCycleCount() < end) {
      __asm__ __volatile__ ("nop");
    };
  }

  return false;
}

// run as soon as possible
void EscDriverEsp32c3::_isr_start(EscDriverTimer timer)
{
  _isr_wait(timer, TIMER_WAIT_EDGE + 10);
  //timer_start((timer_group_t)timer, ESC_TIMER_IDX);
}

void EscDriverEsp32c3::_isr_end(EscDriverTimer timer, void* p)
{
  timer_pause((timer_group_t)timer, ESC_TIMER_IDX);
  timer_disable_intr((timer_group_t)timer, ESC_TIMER_IDX);
}

int EscDriverEsp32c3::attach(size_t channel, int pin, int pulse)
{
  if(channel < 0 || channel >= ESC_CHANNEL_COUNT) return 0;
  _slots[channel].pin = pin;
  _slots[channel].pulse = usToTicks(pulse);
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  return 1;
}

int EscDriverEsp32c3::write(size_t channel, int pulse)
{
  if(channel < 0 || channel >= ESC_CHANNEL_COUNT) return 0;
  _slots[channel].pulse = usToTicks(pulse);
  return 1;
}

int EscDriverEsp32c3::pin(size_t channel) const
{
  if(channel < 0 || channel >= ESC_CHANNEL_COUNT) return -1;
  return _slots[channel].pin;
}

uint32_t EscDriverEsp32c3::telemetry(size_t channel) const
{
  return 0;
}

void EscDriverEsp32c3::apply()
{
  if(_protocol == ESC_PROTOCOL_DISABLED) return;
  if(_protocol >= ESC_PROTOCOL_DSHOT150)
  {
    dshotWrite();
    return;
  }
  if(_async || _busy) return;
  _isr_start(_timer);
}

bool EscDriverEsp32c3::handle(void * p)
{
  // Time critical section
  EscDriver * instance = (EscDriver *)p;

  //instance->_it = NULL;

  if(!instance->_it)
  {
    instance->_busy = true;
    instance->commit();
    instance->_it = instance->_items;
  }

  while(instance->_it && instance->_it != instance->_end)
  {
    const uint32_t ticks = instance->_it->ticks;
    if(instance->_it->clr_mask)
    {
      GPIO.out_w1tc.out_w1tc = instance->_it->clr_mask;
    }
    if(instance->_it->set_mask)
    {
      GPIO.out_w1ts.out_w1ts = instance->_it->set_mask;
    }

    if(instance->_it->last)
    {
      instance->_it = NULL;
      instance->_busy = false;
      if(ticks) _isr_wait(instance->_timer, ticks);
      break;
    }
    instance->_it++;
    if(_isr_wait(instance->_timer, ticks)) break;
  }
  // return whether we need to yield at the end of ISR
  return false;
}

void EscDriverEsp32c3::commit()
{
  Slot sorted[ESC_CHANNEL_COUNT];
  std::copy(_slots, _slots + ESC_CHANNEL_COUNT, sorted);
  std::sort(sorted, sorted + ESC_CHANNEL_COUNT);

  Slot * end = sorted + ESC_CHANNEL_COUNT;

  for(Item * it = _items; it != _items + ESC_CHANNEL_COUNT * 2; ++it) *it = Item();
  Item * item = _items;

  for(Slot * it = sorted; it != end; ++it)
  {
    if(!it->active()) break;
    if(!(_protocol == ESC_PROTOCOL_BRUSHED && it->pulse <= _intervalMin))
    {
      item->set_mask |= (1 << it->pin) & 0xffff;
    }
    if(it == sorted) item->ticks = it->pulse;
  }
  item++;

  Slot * last = NULL;
  for(Slot * it = sorted; it != end; ++it)
  {
    if(!it->active()) break;
    if(last)
    {
      if(!(_protocol == ESC_PROTOCOL_BRUSHED && last->pulse >= _intervalMax))
      {
        item->clr_mask |= (1 << last->pin) & 0xffff;
      }
      uint32_t delta = it->pulse - last->pulse;
      if(delta > DELTA_TICKS_MIN && delta < DELTA_TICKS_MAX)
      {
        item->ticks = delta;
        item++;
      }
    }
    last = it;
  }

  // terminator
  item->last = true;
  if(last)
  {
    if(!(_protocol == ESC_PROTOCOL_BRUSHED && last->pulse >= _intervalMax))
    {
      item->clr_mask |= (1 << last->pin);
    }
  }
  if(_async)
  {
    item->ticks = _interval;
    if(last)
    {
      const int32_t mt = minTicks(_timer);
      if(item->ticks > last->pulse + mt) item->ticks -= last->pulse;
      else item->ticks = mt;
    }
  }
}

uint32_t EscDriverEsp32c3::usToTicksReal(EscDriverTimer timer, uint32_t us)
{
  return (APB_CLK_FREQ / ESC_TIMER_DIVIDER / 1000000L) * us;
}

int32_t EscDriverEsp32c3::minTicks(EscDriverTimer timer)
{
  return 150L;
}

uint32_t EscDriverEsp32c3::usToTicks(uint32_t us)
{
  uint32_t ticks = 0;
  switch(_protocol)
  {
    case ESC_PROTOCOL_ONESHOT125:
      ticks = map(us, 1000, 2000, usToTicksReal(_timer, 125), usToTicksReal(_timer, 250));
      break;
    case ESC_PROTOCOL_ONESHOT42:
      ticks = map(us, 1000, 2000, usToTicksReal(_timer, 42), usToTicksReal(_timer, 84));
      break;
    case ESC_PROTOCOL_MULTISHOT:
      ticks = map(us, 1000, 2000, usToTicksReal(_timer, 5), usToTicksReal(_timer, 25));
      break;
    case ESC_PROTOCOL_BRUSHED:
      ticks = map(constrain(us, 1000, 2000), 1000, 2000, 0, _interval); // strange behaviour at bonduaries
      break;
    case ESC_PROTOCOL_DSHOT150:
    case ESC_PROTOCOL_DSHOT300:
    case ESC_PROTOCOL_DSHOT600:
      ticks = us;
      break;
    default:
      ticks = usToTicksReal(_timer, us); // PWM
      break;
  }
  return ticks;
}

EscDriverEsp32c3::EscDriverEsp32c3(): _busy(false), _async(true), _protocol(ESC_PROTOCOL_PWM), _rate(50), _timer(ESC_DRIVER_TIMER1), _interval(usToTicksReal(_timer, 1000000L / _rate)), _it(NULL), _end(_items + ESC_CHANNEL_COUNT * 2)
{
  for(size_t i = 0; i < ESC_CHANNEL_COUNT; ++i) _slots[i] = Slot();
  for(size_t i = 0; i < ESC_CHANNEL_COUNT * 2; ++i) _items[i] = Item();
}

int EscDriverEsp32c3::begin(const EscConfig& conf)
{
  _protocol = ESC_PROTOCOL_SANITIZE(conf.protocol);
  _async = _protocol == ESC_PROTOCOL_BRUSHED ? true : conf.async; // force async for brushed
  _rate = constrain(conf.rate, 50, 8000);
  _timer = (EscDriverTimer)conf.timer;
  _interval = usToTicksReal(_timer, 1000000L / _rate)/* - 400*/; // small compensation to keep frequency
  _intervalMin = _interval / 500; // do not generate brushed pulses if duty < ~0.2%  (1002)
  _intervalMax = _interval - _intervalMin; // set brushed output hi if duty > ~99.8% (1998)
  switch(_protocol)
  {
    // pulse delays experimentally selected
    case ESC_PROTOCOL_DSHOT150:
      {
        _dh = 69;
        _dl = 57;
      }
      break;
    case ESC_PROTOCOL_DSHOT300:
      {
        _dh = 32;
        _dl = 26;
      }
      break;
    case ESC_PROTOCOL_DSHOT600:
    case ESC_PROTOCOL_PROSHOT:
      {
        _dh = 14;
        _dl = 10;
      }
      break;
    case ESC_PROTOCOL_DISABLED:
      break;
    default: // analog
      _isr_init(_timer, this);
      if(_async) _isr_start(_timer);
  }
  return 1;
}

void EscDriverEsp32c3::end()
{
  if(_protocol < ESC_PROTOCOL_DSHOT150) // analog
  {
    _isr_end(_timer, this);
  }
  for(size_t i = 0; i < ESC_CHANNEL_COUNT; ++i)
  {
    if(_slots[i].pin == -1) continue;
    digitalWrite(_slots[i].pin, LOW);
  }
  _protocol = ESC_PROTOCOL_DISABLED;
}

static inline void dshotDelay(int delay)
{
  while(delay--) __asm__ __volatile__ ("nop");
}

void EscDriverEsp32c3::dshotWrite()
{
  // zero mask arrays
  mask_t * sm = dshotSetMask;
  mask_t * cm = dshotClrMask;
  for(size_t i = 0; i < DSHOT_BIT_COUNT; i++)
  {
    *sm = 0; sm++;
    *cm = 0; cm++;
    *cm = 0; cm++;
  }

  // compute bits
  for(size_t c = 0; c < ESC_CHANNEL_COUNT; c++)
  {
    if(_slots[c].pin > 16 || _slots[c].pin < 0) continue;
    mask_t mask = (1U << _slots[c].pin);
    int pulse = constrain(_slots[c].pulse, 0, 2000);
    // scale to dshot commands (0 or 48-2047)
    int value = dshotConvert(pulse);
    uint16_t frame = dshotEncode(value);
    for(size_t i = 0; i < DSHOT_BIT_COUNT; i++)
    {
      int val = (frame >> (DSHOT_BIT_COUNT - 1 - i)) & 0x01;
      dshotSetMask[i] |= mask;
      dshotClrMask[(i << 1) + val] |= mask;
    }
  }

  // write output
  sm = dshotSetMask;
  cm = dshotClrMask;
  for(size_t i = 0; i < DSHOT_BIT_COUNT; i++)
  {
    GPIO.out_w1ts.out_w1ts = *sm; sm++; dshotDelay(_dh);
    GPIO.out_w1tc.out_w1tc = *cm; cm++; dshotDelay(_dh);
    GPIO.out_w1tc.out_w1tc = *cm; cm++; dshotDelay(_dl);
  }
}

#endif // ESP32c3
