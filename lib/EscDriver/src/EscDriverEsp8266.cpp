#if defined(ESP8266)

#include "EscDriverEsp8266.h"
#include <algorithm>
#include <user_interface.h>

void EscDriverEsp8266::_isr_init(EscDriverTimer timer, void * driver)
{
  switch(timer)
  {
    case ESC_DRIVER_TIMER2:
      ETS_INTR_LOCK();
      ETS_INTR_DISABLE(ETS_FRC_TIMER2_INUM);
      T2C = 0;
      T2I = 0;
      ets_isr_attach(ETS_FRC_TIMER2_INUM, EscDriverEsp8266::handle, driver);
      ETS_INTR_ENABLE(ETS_FRC_TIMER2_INUM);
      T2C = (1 << TCTE) | (TIM_DIV1 << TCPD) | (TIM_EDGE << TCIT) | (TIM_SINGLE << TCAR);
      T2I = 0;
      ETS_INTR_UNLOCK();
      break;
    case ESC_DRIVER_TIMER1:
      ETS_INTR_LOCK();
      ETS_INTR_DISABLE(ETS_FRC_TIMER1_INUM);
      T1C = 0;
      T1I = 0;
      ets_isr_attach(ETS_FRC_TIMER1_INUM, EscDriverEsp8266::handle, driver);
      ETS_INTR_ENABLE(ETS_FRC_TIMER1_INUM);
      T1C = (1 << TCTE) | (TIM_DIV1 << TCPD) | (TIM_EDGE << TCIT) | (TIM_SINGLE << TCAR);
      T1I = 0;
      ETS_INTR_UNLOCK();
      break;
    case ESC_DRIVER_TIMER0:
      ETS_INTR_LOCK();
      ETS_INTR_DISABLE(ETS_CCOMPARE0_INUM);
      ets_isr_attach(ETS_CCOMPARE0_INUM, EscDriverEsp8266::handle, driver);
      ETS_INTR_ENABLE(ETS_CCOMPARE0_INUM);
      ETS_INTR_UNLOCK();
      break;
  }
}

void EscDriverEsp8266::_isr_begin(EscDriverTimer timer)
{
  switch(timer)
  {
    case ESC_DRIVER_TIMER2:
      T2I = 0;
      break;
    case ESC_DRIVER_TIMER1:
      TEIE &= ~TEIE1; //14
      T1I = 0; //9
      break;
    default:
      break;
  }
}

#define TIMER0_WAIT_SHORT_COMP 10UL
#define TIMER0_WAIT_EDGE 240UL
#define TIMER0_WAIT_COMP 200UL
#define TIMER1_WAIT_EDGE 140UL
#define TIMER1_WAIT_COMP 115UL

bool EscDriverEsp8266::_isr_wait(EscDriverTimer timer, const uint32_t ticks)
{
  switch(timer)
  {
    case ESC_DRIVER_TIMER2:
      //if(ticks > 128) { // yield
      if(ticks > TIMER1_WAIT_EDGE) { // yield
        T2A = ((uint32_t)T2V + ticks - TIMER1_WAIT_COMP);
        //T2A = ((uint32_t)T2V + ticks) - 120UL;
        //T2L = 0;
        //T2A = (ticks - 100UL);
        T2I = 0;
        return true;
      }
      break;
    case ESC_DRIVER_TIMER1:
      if(ticks > TIMER1_WAIT_EDGE) { // yield
        T1L = ((ticks - TIMER1_WAIT_COMP) & 0x7FFFFF); //23
        TEIE |= TEIE1; //13
        return true;
      }
      break;
    case ESC_DRIVER_TIMER0:
      if(ticks > TIMER0_WAIT_EDGE) { // yield
        timer0_write((ESP.getCycleCount() + ticks - TIMER0_WAIT_COMP));
        return true;
      }
      break;
  }

  if(ticks > 20) { // or delay
    const uint32_t end = ESP.getCycleCount() + ticks - TIMER0_WAIT_SHORT_COMP;
    while(ESP.getCycleCount() < end) {
      __asm__ __volatile__ ("nop");
    };
  }

  return false;
}

// run as soon as possible
void EscDriverEsp8266::_isr_start(EscDriverTimer timer)
{
  switch(timer)
  {
    case ESC_DRIVER_TIMER2:
      //T2L = 0;
      T2A = (uint32_t)T2V + 120UL;
      //T2I = 0;
      break;
    case ESC_DRIVER_TIMER1:
      T1L = 120UL;
      TEIE |= TEIE1;
      break;
    case ESC_DRIVER_TIMER0:
      timer0_write(ESP.getCycleCount() + 200UL);
      break;
  }
}

void EscDriverEsp8266::_isr_reboot(void* p)
{
  EscDriver* d = (EscDriver*)p;
  _isr_begin(d->_timer);
}

void EscDriverEsp8266::_isr_end(EscDriverTimer timer, void* p)
{
  switch(timer)
  {
    case ESC_DRIVER_TIMER2:
      ETS_INTR_LOCK();
      ETS_INTR_DISABLE(ETS_FRC_TIMER2_INUM);
      //ets_isr_attach(ETS_FRC_TIMER2_INUM, _isr_reboot, p);
      //ETS_INTR_ENABLE(ETS_FRC_TIMER2_INUM);
      ETS_INTR_UNLOCK();
      break;
    case ESC_DRIVER_TIMER1:
      ETS_INTR_LOCK();
      ETS_INTR_DISABLE(ETS_FRC_TIMER1_INUM);
      ETS_INTR_UNLOCK();
      break;
    case ESC_DRIVER_TIMER0:
      ETS_INTR_LOCK();
      ETS_INTR_DISABLE(ETS_CCOMPARE0_INUM);
      ETS_INTR_UNLOCK();
      break;
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

int EscDriverEsp8266::pin(size_t channel) const
{
  if(channel < 0 || channel >= ESC_CHANNEL_COUNT) return -1;
  return _slots[channel].pin;
}

uint32_t EscDriverEsp8266::telemetry(size_t channel) const
{
  return 0;
}

int EscDriverEsp8266::write(size_t channel, int pulse)
{
  if(channel < 0 || channel >= ESC_CHANNEL_COUNT) return 0;
  _slots[channel].pulse = usToTicks(pulse);
  return 1;
}

void EscDriverEsp8266::apply()
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

void EscDriverEsp8266::handle(void * p, void * x)
{
  // Time critical section
  EscDriver * instance = (EscDriver *)p;
  _isr_begin(instance->_timer);

  //instance->_it = NULL;

  if(!instance->_it)
  {
    instance->_busy = true;
    instance->commit();
    instance->_it = instance->_items;
  }

  while(instance->_it && instance->_it != instance->_end)
  {
    uint32_t ticks = instance->_it->ticks;
    if(instance->_it->clr_mask)
    {
      GPOC = instance->_it->clr_mask;
    }
    if(instance->_it->set_mask)
    {
      GPOS = instance->_it->set_mask;
    }

    if(instance->_it->last)
    {
      instance->_it = NULL;
      instance->_busy = false;
      if(ticks) _isr_wait(instance->_timer, ticks);
      break;
    }
    else
    {
      instance->_it++;
      if(_isr_wait(instance->_timer, ticks)) break;
    }
  }
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
      const int32_t mt = minTicks(_timer);
      if(item->ticks > last->pulse + mt) item->ticks -= last->pulse;
      else item->ticks = mt;
    }
  }
}

uint32_t EscDriverEsp8266::usToTicksReal(EscDriverTimer timer, uint32_t us)
{
  switch(timer)
  {
    case ESC_DRIVER_TIMER0:
      return (F_CPU / 1000000L) * us;
    default:
      return (APB_CLK_FREQ / 1000000L) * us;
  }
}

int32_t EscDriverEsp8266::minTicks(EscDriverTimer timer)
{
  switch(timer)
  {
    case ESC_DRIVER_TIMER0:
      return 250L;
    default:
      return 150L;
  }
}

uint32_t EscDriverEsp8266::usToTicks(uint32_t us)
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

EscDriverEsp8266::EscDriverEsp8266(): _busy(false), _async(true), _protocol(ESC_PROTOCOL_PWM), _rate(50), _timer(ESC_DRIVER_TIMER1), _interval(usToTicksReal(_timer, 1000000L / _rate)), _it(NULL), _end(_items + ESC_CHANNEL_COUNT * 2)
{
  for(size_t i = 0; i < ESC_CHANNEL_COUNT; ++i) _slots[i] = Slot();
  for(size_t i = 0; i < ESC_CHANNEL_COUNT * 2; ++i) _items[i] = Item();
}

int EscDriverEsp8266::begin(const EscConfig& conf)
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
        _dh = 60;
        _dm = 61;
        _dl = 44;
      }
      break;
    case ESC_PROTOCOL_DSHOT300:
      {
        _dh = 30;
        _dm = 29;
        _dl = 21;
      }
      break;
    case ESC_PROTOCOL_DSHOT600:
    case ESC_PROTOCOL_PROSHOT:
      {
        _dh = 14;
        _dm = 14;
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

void EscDriverEsp8266::end()
{
  if(_protocol < ESC_PROTOCOL_DSHOT150) // analog
  {
    _isr_end(_timer, this);
  }
  _protocol = ESC_PROTOCOL_DISABLED;
  for(size_t i = 0; i < ESC_CHANNEL_COUNT; ++i)
  {
    if(_slots[i].pin == -1) continue;
    digitalWrite(_slots[i].pin, LOW);
  }
}

static IRAM_ATTR void dshotDelay(int delay)
{
  while(delay--) __asm__ __volatile__ ("nop");
}

void EscDriverEsp8266::dshotWrite()
{
  // zero mask arrays
  mask_t smask[DSHOT_BIT_COUNT];
  mask_t cmask[DSHOT_BIT_COUNT];
  std::fill_n(smask, DSHOT_BIT_COUNT, 0);
  std::fill_n(cmask, DSHOT_BIT_COUNT, 0);

  // compute bits
  for(size_t c = 0; c < ESC_CHANNEL_COUNT; c++)
  {
    if(_slots[c].pin > 15 || _slots[c].pin < 0) continue;
    mask_t mask = (1U << _slots[c].pin);
    int pulse = constrain(_slots[c].pulse, 0, 2000);
    // scale to dshot commands (0 or 48-2047)
    int value = dshotConvert(pulse);
    uint16_t frame = dshotEncode(value);
    for(size_t i = 0; i < DSHOT_BIT_COUNT; i++)
    {
      int val = (frame >> (DSHOT_BIT_COUNT - 1 - i)) & 0x01;
      smask[i] |= mask;
      cmask[i] |= val ? 0 : mask;
    }
  }

  // write output
  mask_t * sm = smask;
  mask_t * cm = cmask;
  for(size_t i = 0; i < DSHOT_BIT_COUNT; i++)
  {
    GPOS = *sm; dshotDelay(_dh);
    GPOC = *cm; dshotDelay(_dm);
    GPOC = *sm; dshotDelay(_dl);
    sm++;
    cm++;
  }
}

#endif // ESP8266
