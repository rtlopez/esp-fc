#if defined(ESP8266)

#include "EscDriverEsp8266.h"
#include <algorithm>
#include <user_interface.h>

#if defined(USE_FRC1_NMI)
static void ICACHE_RAM_ATTR _isr_nmi()
{
  EscDriverEsp8266::handle(NULL);
}
#endif

static void _isr_init()
{
#if defined(USE_FRC2)
  ETS_INTR_LOCK();
  ETS_INTR_DISABLE(ETS_FRC_TIMER2_INUM);
  T2C = 0;
  T2I = 0;
  ets_isr_attach(ETS_FRC_TIMER2_INUM, EscDriverEsp8266::handle, NULL);
  ETS_INTR_ENABLE(ETS_FRC_TIMER2_INUM);
  T2C = (1 << TCTE) | (TIM_DIV1 << TCPD) | (TIM_EDGE << TCIT) | (TIM_SINGLE << TCAR);
  T2I = 0;
  ETS_INTR_UNLOCK();
#elif defined(USE_FRC1)
  ETS_INTR_LOCK();
  ETS_INTR_DISABLE(ETS_FRC_TIMER1_INUM);
  T1C = 0;
  T1I = 0;
  #if defined(USE_FRC1_NMI)
    NmiTimSetFunc(_isr_nmi);
  #else
    ets_isr_attach(ETS_FRC_TIMER1_INUM, EscDriverEsp8266::handle, NULL);
  #endif
  ETS_INTR_ENABLE(ETS_FRC_TIMER1_INUM);
  T1C = (1 << TCTE) | (TIM_DIV1 << TCPD) | (TIM_EDGE << TCIT) | (TIM_SINGLE << TCAR);
  T1I = 0;
  ETS_INTR_UNLOCK();
#elif defined(USE_FRC0)
  ETS_INTR_LOCK();
  ETS_INTR_DISABLE(ETS_CCOMPARE0_INUM);
  ets_isr_attach(ETS_CCOMPARE0_INUM, EscDriverEsp8266::handle, NULL);
  ETS_INTR_ENABLE(ETS_CCOMPARE0_INUM);
  ETS_INTR_UNLOCK();
#elif defined(USE_UNIT)
#else
  #error "Missing timer definition"
#endif
}

static void ICACHE_RAM_ATTR _isr_begin()
{
#if defined(USE_FRC2)
  T2I = 0;
#elif defined(USE_FRC1)
  TEIE &= ~TEIE1; //14
  T1I = 0; //9
#elif defined(USE_FRC0)
#elif defined(USE_UNIT)
#else
  #error "Missing timer definition"
#endif
}

static bool ICACHE_RAM_ATTR _isr_wait(const uint32_t ticks)
{
#if defined(USE_FRC2)
  //if(ticks > 128) { // yield
  if(ticks > 140) { // yield
    //T2A = ((uint32_t)T2V + ticks - 108UL);
    //T2A = ((uint32_t)T2V + ticks) - 120UL;
    T2L = 0;
    T2A = ticks - 120UL;
    T2I = 0;
    return true;
  }
#elif defined(USE_FRC1)
  if(ticks > 140) { // yield
    T1L = ((ticks - 120UL) & 0x7FFFFF); //23
    TEIE |= TEIE1; //13
    return true;
  }
#elif defined(USE_FRC0)
  if(ticks > 240) { // yield
    timer0_write((ESP.getCycleCount() + ticks - 200UL));
    return true;
  }
#elif defined(USE_UNIT)
#else
  #error "Missing timer definition"
#endif

#if !defined(USE_UNIT)
  if(ticks > 20) { // or delay
    const uint32_t now = ESP.getCycleCount();
    const uint32_t end = now + ticks - 10;
    while(ESP.getCycleCount() < end) {
      __asm__ __volatile__ ("nop");
    };
  }
#endif

  return false;
}

// run as soon as possible
static void ICACHE_RAM_ATTR _isr_start()
{
#if defined(USE_FRC2)
  //T2L = 0;
  T2A = (uint32_t)T2V + 100UL;
  //T2I = 0;
#elif defined(USE_FRC1)
  T1L = 100UL;
  TEIE |= TEIE1;
#elif defined(USE_FRC0)
  timer0_write(ESP.getCycleCount() + 200UL);
#elif defined(USE_UNIT)
#else
  #error "Missing timer definition"
#endif
}

static void ICACHE_RAM_ATTR _isr_reboot(void* p)
{
  _isr_begin();
  (void)p;
}

static void _isr_end()
{
#if defined(USE_FRC2)
  ETS_INTR_LOCK();
  ETS_INTR_DISABLE(ETS_FRC_TIMER2_INUM);
  ets_isr_attach(ETS_FRC_TIMER2_INUM, _isr_reboot, NULL);
  ETS_INTR_ENABLE(ETS_FRC_TIMER2_INUM);
  ETS_INTR_UNLOCK();
#elif defined(USE_FRC1)
  ETS_INTR_LOCK();
  ETS_INTR_DISABLE(ETS_FRC_TIMER1_INUM);
  ETS_INTR_UNLOCK();
#elif defined(USE_FRC0)
  ETS_INTR_LOCK();
  ETS_INTR_DISABLE(ETS_CCOMPARE0_INUM);
  ETS_INTR_UNLOCK();
#elif defined(USE_UNIT)
#else
  #error "Missing timer definition"
#endif
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

int ICACHE_RAM_ATTR EscDriverEsp8266::write(size_t channel, int pulse)
{
  if(channel < 0 || channel >= ESC_CHANNEL_COUNT) return 0;
  _slots[channel].pulse = usToTicks(pulse);
  return 1;
}

void ICACHE_RAM_ATTR EscDriverEsp8266::apply()
{
  if(_protocol >= ESC_PROTOCOL_DSHOT150)
  {
    dshotWrite();
    return;
  }
  if(_async || _busy) return;
  _isr_start();
}

void ICACHE_RAM_ATTR EscDriverEsp8266::handle(void * p)
{
  // Time critical section
  _isr_begin();

  static Item * it = NULL;
  static const Item * end = _instance->_items + ESC_CHANNEL_COUNT * 2;

  if(!it)
  {
    _instance->_busy = true;
    _instance->commit();
    it = _instance->_items;
  }

  while(it && it != end)
  {
    uint32_t ticks = it->ticks;
    if(it->clr_mask)
    {
      if(it->clr_mask & 0xffff)   GPOC = (it->clr_mask & 0xffff);
      if(it->clr_mask & 0x10000) GP16O = 0;
    }
    if(it->set_mask)
    {
      if(it->set_mask & 0xffff)   GPOS = (it->set_mask & 0xffff);
      if(it->set_mask & 0x10000) GP16O = 1;
    }

    if(it->last)
    {
      it = NULL;
      _instance->_busy = false;
      if(ticks) _isr_wait(ticks);
      break;
    }
    else
    {
      it++;
      if(_isr_wait(ticks)) break;
    }
  }
}

void ICACHE_RAM_ATTR EscDriverEsp8266::commit()
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
      item->set_mask |= (1 << it->pin);
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
        item->clr_mask |= (1 << last->pin);
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
      if(item->ticks > last->pulse + TIMER_TICKS_MIN) item->ticks -= last->pulse;
      else item->ticks = TIMER_TICKS_MIN;
    }
  }
}

uint32_t ICACHE_RAM_ATTR EscDriverEsp8266::usToTicks(uint32_t us)
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
      ticks = map(constrain(us, 1000, 2000), 1000, 2000, 0, _interval); // strange behaviour at bonduaries
      break;
    case ESC_PROTOCOL_DSHOT150:
    case ESC_PROTOCOL_DSHOT300:
    case ESC_PROTOCOL_DSHOT600:
    case ESC_PROTOCOL_DSHOT1200:
      ticks = us;
      break;
    default:
      ticks = usToTicksReal(us); // PWM
      break;
  }
  return ticks;
}

EscDriverEsp8266::EscDriverEsp8266(): _busy(false), _async(true), _protocol(ESC_PROTOCOL_PWM), _rate(50), _interval(usToTicksReal(1000000L / _rate))
{
  for(size_t i = 0; i < ESC_CHANNEL_COUNT; ++i) _slots[i] = Slot();
  for(size_t i = 0; i < ESC_CHANNEL_COUNT * 2; ++i) _items[i] = Item();
  _instance = this;
}

int EscDriverEsp8266::begin(EscProtocol protocol, bool async, int16_t rate)
{
  _protocol = ESC_PROTOCOL_SANITIZE(protocol);
  _async = _protocol == ESC_PROTOCOL_BRUSHED ? true : async; // force async for brushed
  _rate = constrain(rate, 50, 8000);
  _interval = usToTicksReal(1000000L / _rate)/* - 400*/; // small compensation to keep frequency
  _intervalMin = _interval / 500; // do not generate brushed pulses if duty < ~0.2%  (1002)
  _intervalMax = _interval - _intervalMin; // set brushed output hi if duty > ~99.8% (1998)
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
      _isr_init();
      if(_async) _isr_start();
  }
  return 1;
}

void EscDriverEsp8266::end()
{
  if(_protocol < ESC_PROTOCOL_DSHOT150) // analog
  {
    _isr_end();
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

uint16_t EscDriverEsp8266::dshotEncode(uint16_t value)
{
  value <<= 1;

  // compute checksum
  int csum = 0;
  int csum_data = value;
  for (int i = 0; i < 3; i++)
  {
    csum ^= csum_data; // xor
    csum_data >>= 4;
  }
  csum &= 0xf;

  return (value << 4) | csum;
}

EscDriverEsp8266 * EscDriverEsp8266::_instance = NULL;

#endif // ESP8266
