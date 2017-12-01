#include "EspSoftSerial.h"

#define USE_TIMER0 1
#define TIMER0_US_CLOCKS (F_CPU / 1000000L)
#define TIMER1_US_CLOCKS (APB_CLK_FREQ / 1000000L)

uint32_t EspSoftSerial::nsToTicks(uint32_t ns)
{
#ifdef USE_TIMER0
  return (ns * TIMER0_US_CLOCKS) / 1000L; // timer0
#else
  return (ns * TIMER1_US_CLOCKS) / 1000L; // timer1
#endif
}

void EspSoftSerial::timerInit()
{
  noInterrupts();
#ifdef USE_TIMER0
  timer0_isr_init();
  timer0_attachInterrupt(&EspSoftSerial::rx_read_bit_isr);
  timer0_write(ESP.getCycleCount() + nsToTicks(10000000)); // 10ms
#else
  timer1_isr_init();
  timer1_attachInterrupt(&EspSoftSerial::rx_read_bit_isr);
  timer1_enable(TIM_DIV1, TIM_EDGE, TIM_SINGLE);
#endif
  interrupts();
}

void EspSoftSerial::timerStop()
{
  noInterrupts();
#ifdef USE_TIMER0
  timer0_detachInterrupt();
#else
  timer1_disable();
  timer1_detachInterrupt();
#endif
  interrupts();
}

void EspSoftSerial::timerWrite(uint32_t ticks)
{
#ifdef USE_TIMER0
  timer0_write(ESP.getCycleCount() + ticks);
#else
  timer1_write(ticks);
#endif
}

void EspSoftSerial::rxReadBit()
{
  bool v = EspGpio::digitalRead(_conf.rx_pin);
  v = _conf.inverted ? !v : v;
  switch(_rx_state)
  {
    case STATE_START:
      timerWrite(_delay);
      _rx_state = STATE_DATA;
      break;

    case STATE_DATA:
      timerWrite(_delay);
      _rx_data >>= 1;
      if(v) _rx_data |= 0x80;
      if(++_rx_data_count >= _conf.data_bits)
      {
        _rx_data_count = 0;
        if(_conf.parity_type) _rx_state = STATE_PARITY;
        else _rx_state = STATE_STOP;
      }
      break;

    case STATE_PARITY:
      // TODO: validate parity
      timerWrite(_delay);
      _rx_state = STATE_STOP;
      break;

    case STATE_STOP:
      _rx_buff.push(_rx_data);
      _rx_state = STATE_IDLE;
      // no break here, call idle

    case STATE_IDLE:
    default:
      timerWrite(_delay * 5000); // avoid wdt reset and wait ~5k bits for rxStart() call
      break;
  }
}

void EspSoftSerial::rxStart()
{
  switch(_rx_state)
  {
    case STATE_IDLE:
      _rx_state = STATE_START;
      if(_delay_start > 200) { // needs at least 200 cpu clocks
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
  if(_instance)
  {
    timerStop();
    detachInterrupt(_conf.rx_pin);
    _instance = NULL;
  }
  if(conf.rx_pin != -1 && conf.baud > 0)
  {
    _instance = this;
    _conf = conf;
    _delay = nsToTicks((1000000000L / _conf.baud) - 1200);
    _delay_start = (_delay >> 1); // start bit half delay
    if(_delay_start > nsToTicks(2700) + 300) // leave at least 300 cpu clocks margin
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

    return 1;
  }
  return 0;
}

EspSoftSerial * EspSoftSerial::_instance = NULL;
