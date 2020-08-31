#ifndef _ESPFC_DEBUG_H_
#define _ESPFC_DEBUG_H_

#include <EspGpio.h>
#include <Arduino.h>

namespace Espfc
{

#if 0
#define PIN_DEBUG(v) EspGpio::digitalWrite(D0, v)
#define PIN_DEBUG_INIT() pinMode(D0, OUTPUT)
#else
#define PIN_DEBUG(v)
#define PIN_DEBUG_INIT()
#endif

#if defined(ESPFC_SERIAL_DEBUG)
static Stream * _debugStream = nullptr;

#define LOG_SERIAL_INIT(p) _debugStream = p;
#define LOG_SERIAL_DEBUG(v) if(_debugStream) { _debugStream->print(' '); _debugStream->print(v); }

template <typename T>
void D(T t)
{
  if(_debugStream)
  {
    _debugStream->print(t);
    _debugStream->print('\n');
  }
}

template<typename T, typename... Args>
void D(T t, Args... args) // recursive variadic function
{
  if(_debugStream)
  {
    _debugStream->print(t);
    _debugStream->print(' ');
    D(args...);
  }
}

#else

#define LOG_SERIAL_INIT(p)
#define LOG_SERIAL_DEBUG(v)
#define D(...)

#endif

}

#endif
