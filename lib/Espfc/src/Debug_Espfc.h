#pragma once

#include "Stream/Printer.hpp"

#ifdef ESPFC_DEBUG_PIN
#include "Hal/Gpio.hpp"
#define PIN_DEBUG(v) ::Espfc::Hal::Gpio::digitalWrite(ESPFC_DEBUG_PIN, v)
#define PIN_DEBUG_INIT() ::Espfc::Hal::Gpio::pinMode(ESPFC_DEBUG_PIN, OUTPUT)
#else
#define PIN_DEBUG(v)
#define PIN_DEBUG_INIT()
#endif

namespace Espfc {

void initDebugStream(Stream::Printer p);
extern Stream::Printer _debugStream;

#ifdef ESPFC_DEBUG_SERIAL

#define LOG_SERIAL_DEBUG(v) _debugStream.print(v);
#define LOG_SERIAL_DEBUG_HEX(v) _debugStream.print(v, HEX);

template<typename T>
void D(T t)
{
  _debugStream.println(t);
}

// recursive variadic function
template<typename T, typename... Args>
void D(T t, Args... args)
{
  _debugStream.print(t);
  _debugStream.print(' ');
  D(args...);
}

#else

#define LOG_SERIAL_DEBUG(v)
#define LOG_SERIAL_DEBUG_HEX(v)
#define D(...)

#endif

} // namespace Espfc
