#include "EspSoftSerial.h"

#if defined(ESP8266)

#define TIMER0_US_CLOCKS (F_CPU / 1000000L)
#define TIMER1_US_CLOCKS (APB_CLK_FREQ / 1000000L)

//#define USE_TIMER0
#define USE_TIMER1
#define USE_TIMER1_NMI

#if defined(USE_TIMER0)
  #define TIMER_DELAY_MIN 200
#elif defined(USE_TIMER1)
  #define TIMER_DELAY_MIN 100
#else
  #error "Timer not defined"
#endif

uint32_t EspSoftSerial::nsToTicks(uint32_t ns)
{
#if defined(USE_TIMER0)
  return (ns * TIMER0_US_CLOCKS) / 1000L; // timer0
#elif defined(USE_TIMER1)
  return (ns * TIMER1_US_CLOCKS) / 1000L; // timer1
#else
  #error "Timer not defined"
#endif
}

#if defined(USE_TIMER1) && !defined(USE_TIMER1_NMI)
static void ICACHE_RAM_ATTR _rx_read_bit_isr(void * p)
{
  EspSoftSerial::rx_read_bit_isr();
}
#endif

void EspSoftSerial::timerInit()
{
#if defined(USE_TIMER0)
  noInterrupts();
  timer0_isr_init();
  timer0_attachInterrupt(EspSoftSerial::rx_read_bit_isr);
  timer0_write(ESP.getCycleCount() + nsToTicks(10000000)); // 10ms
  interrupts();
#elif defined(USE_TIMER1)
  ETS_INTR_LOCK();
  ETS_INTR_DISABLE(ETS_FRC_TIMER1_INUM);
  T1C = 0;
  T1I = 0;
  #if defined(USE_TIMER1_NMI)
    NmiTimSetFunc(EspSoftSerial::rx_read_bit_isr);
  #else
    ets_isr_attach(ETS_FRC_TIMER1_INUM, _rx_read_bit_isr, NULL);
  #endif
  ETS_INTR_ENABLE(ETS_FRC_TIMER1_INUM);
  T1C = (1 << TCTE) | (TIM_DIV1 << TCPD) | (TIM_EDGE << TCIT) | (TIM_SINGLE << TCAR);
  T1I = 0;
  ETS_INTR_UNLOCK();
#else
  #error "Timer not defined"
#endif
}

void EspSoftSerial::timerStop()
{
#if defined(USE_TIMER0)
  noInterrupts();
  timer0_detachInterrupt();
  interrupts();
#elif defined(USE_TIMER1)
  ETS_INTR_LOCK();
  ETS_INTR_DISABLE(ETS_FRC_TIMER1_INUM);
  ETS_INTR_UNLOCK();
#else
  #error "Timer not defined"
#endif
}

static void ICACHE_RAM_ATTR timerWrite(uint32_t ticks)
{
#if defined(USE_TIMER0)
  timer0_write(ESP.getCycleCount() + ticks);
#elif defined(USE_TIMER1)
  //timer1_write(ticks);
  T1L = ((ticks) & 0x7FFFFF); //23
  TEIE |= TEIE1; //13
#else
  #error "Timer not defined"
#endif
}

static inline bool ICACHE_RAM_ATTR _readPin(int pin, bool invert)
{
  return (bool)(GPI & (1 << pin)) ^ invert;
  /*int c = 3;
  int v = 0;
  while(c--)
  {
    //v += (bool)GPIP(pin);
    v += (bool)(GPI & (1 << pin));
  }
  v = v >> 1; // calc avg of 3 samples
  return invert ? !v : v;*/
}

void EspSoftSerial::rxReadBit()
{
#if defined(USE_TIMER1)
  TEIE &= ~TEIE1; //14
  T1I = 0; //9
#endif

  //GP16O = 1;
  switch(_rx_state)
  {
    case STATE_DATA:
      timerWrite(_delay);
      _rx_data >>= 1;
      if(_readPin(_conf.rx_pin, _conf.inverted)) _rx_data |= 0x80;
      if(++_rx_data_count >= _conf.data_bits)
      {
        _rx_state =_conf.parity_type ? STATE_PARITY : STATE_STOP;
        _rx_data_count = 0;
      }
      break;

    case STATE_PARITY:
      timerWrite(_delay);
      if(_readPin(_conf.rx_pin, _conf.inverted)) _rx_data |= 0x100;
      _rx_state = STATE_STOP;
      break;

    case STATE_STOP:
      _rx_buff.push((int16_t)(_rx_data & 0xffff));
      _rx_data = 0;
      _rx_state = STATE_IDLE;
      // no break here, call idle

    case STATE_IDLE:
    default:
#if defined(USE_TIMER0)
      timerWrite(_delay * 5000); // avoid wdt reset and wait ~5k bits for rxStart() call
#endif
      break;
  }
  //GP16O = 0;
}

void EspSoftSerial::rxStart()
{
  switch(_rx_state)
  {
    case STATE_IDLE:
      _rx_state = STATE_DATA;
      if(_delay_start > TIMER_DELAY_MIN) { // needs at least 200 cpu clocks
        timerWrite(_delay_start);
      } else {
        rxReadBit();
      }
      break;
    default:
      break;
  }
}

void EspSoftSerial::rx_read_bit_isr()
{
  if(_instance) _instance->rxReadBit();
}

void EspSoftSerial::rx_start_isr()
{
  if(_instance) _instance->rxStart();
}

int EspSoftSerial::begin(int baud)
{
  EspSoftSerialConfig conf(baud, -1);
  begin(conf);
  return 1;
}

int EspSoftSerial::begin(const EspSoftSerialConfig& conf)
{
  if(_instance && _conf.rx_pin != -1)
  {
    timerStop();
    detachInterrupt(_conf.rx_pin);
    _instance = NULL;
  }
  if(conf.rx_pin != -1 && conf.baud > 0)
  {
    _instance = this;
    _conf = conf;
    _delay = nsToTicks((1000000000UL / _conf.baud) - 1500UL); // ~1.5us margin for isr call and computation
    _delay_start = _delay + (_delay >> 1); // start bit half delay
    if(_delay_start > nsToTicks(2700) + TIMER_DELAY_MIN) // leave at least 300 cpu clocks margin
    {
      _delay_start -= nsToTicks(2700); // pin isr call compensation
    }
    else
    {
      _delay_start = 0;
    }
    pinMode(_conf.rx_pin, INPUT);
    attachInterrupt(_conf.rx_pin, &EspSoftSerial::rx_start_isr, _conf.inverted ? RISING : FALLING);
    timerInit();

    //pinMode(16, OUTPUT);

    return 1;
  }
  return 0;
}

EspSoftSerial * EspSoftSerial::_instance = NULL;

#endif //ESP8266
