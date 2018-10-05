#if defined(ESP8266)

#include "EscDriverEsp8266.h"
#include <algorithm>

#define DELTA_TICKS_MAX ((APB_CLK_FREQ / 1000000L) * 50000L)
#define DELTA_TICKS_MIN 5

static void ICACHE_RAM_ATTR handleIsr(void * arg)
{
  // Time critical section
  EscDriver * drv = (EscDriver *)arg;

  if(!drv->it())
  {
    drv->busy(true);
    drv->commit();
    drv->rewind();
  }

  while(drv->working())
  {
    volatile EscDriverEsp8266::Item * it = drv->it();
    uint32_t ticks = it->ticks;
    if(it->clr_mask)
    {
      //if(it->clr_mask & 0xffff)   GPOC = (it->clr_mask & 0xffff);
      //if(it->clr_mask & 0x10000) GP16O = 0;
      GPOC = it->clr_mask;
    }
    if(it->set_mask)
    {
      //if(it->set_mask & 0xffff)   GPOS = (it->set_mask & 0xffff);
      //if(it->set_mask & 0x10000) GP16O = 1;
      GPOS = it->set_mask;
    }

    if(it->last)
    {
      drv->done();
      drv->busy(false);
      if(ticks) drv->wait(ticks);
      break;
    }
    else
    {
      drv->next();
      if(drv->wait(ticks)) break;
    }
  }
}

int EscDriverEsp8266::attach(size_t channel, int pin, int pulse)
{
  if(channel < 0 || channel >= ESC_CHANNEL_COUNT) return 0;
  _slots[channel].pin = pin;
  _slots[channel].pulse = usToTicks(pulse);
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  return 1;
}

int EscDriverEsp8266::write(size_t channel, int pulse)
{
  if(channel < 0 || channel >= ESC_CHANNEL_COUNT) return 0;
  _slots[channel].pulse = usToTicks(pulse);
  return 1;
}

void EscDriverEsp8266::apply()
{
  if(_protocol >= ESC_PROTOCOL_DSHOT150)
  {
    dshotWrite();
    return;
  }
  if(_async || _busy) return;
  _timer.write(300);
}

void EscDriverEsp8266::commit()
{
  Slot sorted[ESC_CHANNEL_COUNT];
  std::copy(_slots, _slots + ESC_CHANNEL_COUNT, sorted);
  std::sort(sorted, sorted + ESC_CHANNEL_COUNT);

  Slot * end = sorted + ESC_CHANNEL_COUNT;
  Slot * last = NULL;

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

  for(Slot * it = sorted; it != end; ++it)
  {
    if(!it->active()) break;
    if(last) {
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
      const int32_t mt = _timer.minTicks();
      if(item->ticks > last->pulse + mt) item->ticks -= last->pulse;
      else item->ticks = mt;
    }
  }
}

uint32_t EscDriverEsp8266::usToTicks(uint32_t us)
{
  uint32_t ticks = 0;
  switch(_protocol)
  {
    case ESC_PROTOCOL_ONESHOT125:
      ticks = map(us, 1000, 2000, _timer.usToTicks(125), _timer.usToTicks(250));
      break;
    case ESC_PROTOCOL_ONESHOT42:
      ticks = map(us, 1000, 2000, _timer.usToTicks(42), _timer.usToTicks(84));
      break;
    case ESC_PROTOCOL_MULTISHOT:
      ticks = map(us, 1000, 2000, _timer.usToTicks(5), _timer.usToTicks(25));
      break;
    case ESC_PROTOCOL_BRUSHED:
      ticks = map(constrain(us, 1000, 2000), 1000, 2000, 0, _interval); // strange behaviour at bonduaries
      break;
    case ESC_PROTOCOL_DSHOT150:
    case ESC_PROTOCOL_DSHOT300:
    case ESC_PROTOCOL_DSHOT600:
    case ESC_PROTOCOL_DSHOT1200:
      ticks = us;
      break;
    default:
      ticks = _timer.usToTicks(us); // PWM
      break;
  }
  return ticks;
}

EscDriverEsp8266::EscDriverEsp8266(): _busy(false), _async(true), _protocol(ESC_PROTOCOL_PWM), _rate(50), _timer(ESC_DRIVER_TIMER1), _interval(_timer.usToTicks(1000000L / _rate)), _it(NULL), _end(_items + ESC_CHANNEL_COUNT * 2)
{
  for(size_t i = 0; i < ESC_CHANNEL_COUNT; ++i) _slots[i] = Slot();
  for(size_t i = 0; i < ESC_CHANNEL_COUNT * 2; ++i) _items[i] = Item();
}

int EscDriverEsp8266::begin(EscProtocol protocol, bool async, int16_t rate, EscDriverTimer timer)
{
  _protocol = ESC_PROTOCOL_SANITIZE(protocol);
  _async = _protocol == ESC_PROTOCOL_BRUSHED ? true : async; // force async for brushed
  _rate = constrain(rate, 50, 8000);
  switch(_protocol)
  {
    // pulse delays experimentally selected
    case ESC_PROTOCOL_DSHOT150:
      {
        const float base = 206;
        _dh = lrintf(base * 0.35f);
        _dl = lrintf(base * 0.30f);
      }
      break;
    case ESC_PROTOCOL_DSHOT300:
      {
        const float base = 101;
        _dh = lrintf(base * 0.35f);
        _dl = lrintf(base * 0.30f);
      }
      break;
    case ESC_PROTOCOL_DSHOT600:
    case ESC_PROTOCOL_DSHOT1200:
    case ESC_PROTOCOL_PROSHOT:
      {
        const float base = 47;
        _dh = lrintf(base * 0.36f);
        _dl = lrintf(base * 0.28f);
      }
      break;
    default: // analog
      _timer.begin(timer, handleIsr, this);
      _interval = _timer.usToTicks(1000000L / _rate)/* - 400*/; // small compensation to keep frequency
      _intervalMin = _interval / 500; // do not generate brushed pulses if duty < ~0.2%  (1002)
      _intervalMax = _interval - _intervalMin; // set brushed output hi if duty > ~99.8% (1998)
      if(async) _timer.write(300);
  }
  return 1;
}

void EscDriverEsp8266::end()
{
  if(_protocol < ESC_PROTOCOL_DSHOT150) // analog
  {
    //_isr_end(_timer, this);
    _timer.end();
  }
  for(size_t i = 0; i < ESC_CHANNEL_COUNT; ++i)
  {
    if(_slots[i].pin == -1) continue;
    digitalWrite(_slots[i].pin, LOW);
  }
}

static inline void dshotDelay(int delay)
{
  while(delay--) __asm__ __volatile__ ("nop");
}

void EscDriverEsp8266::dshotWrite()
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
    int value = 0; // disarmed
    // scale to dshot commands (0 or 48-2047)
    if(pulse > 1000)
    {
      value =  PWM_TO_DSHOT(pulse);
    }
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
    GPOS = *sm; sm++; dshotDelay(_dh);
    GPOC = *cm; cm++; dshotDelay(_dh);
    GPOC = *cm; cm++; dshotDelay(_dl);
  }
}

#endif // ESP8266
