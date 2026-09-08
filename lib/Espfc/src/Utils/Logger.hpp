#pragma once

#include <cstddef>
#include <memory>

#include "Debug_Espfc.h"
#include "Stream/BufferWritable.hpp"
#include "Stream/Printer.hpp"

namespace Espfc::Utils {

class Logger
{
public:
  Logger();

  int begin(size_t size = 2048);

  Logger& info();
  Logger& err();

  template<typename T>
  Logger& log(const T& v)
  {
    LOG_SERIAL_DEBUG(' ')
    LOG_SERIAL_DEBUG(v)
    _printer.print(' ');
    _printer.print(v);
    return *this;
  }

  template<typename T>
  Logger& loghex(const T& v)
  {
    LOG_SERIAL_DEBUG(' ')
    LOG_SERIAL_DEBUG_HEX(v)
    _printer.print(' ');
    _printer.print(v, HEX);
    return *this;
  }

  template<typename T>
  Logger& logln(const T& v)
  {
    LOG_SERIAL_DEBUG(' ')
    LOG_SERIAL_DEBUG(v)
    _printer.print(' ');
    _printer.print(v);
    return endl();
  }

  Logger& endl();

  const char* c_str() const;
  size_t length() const;

private:
  std::unique_ptr<char[]> _buff;
  Stream::BufferWritable _sink;
  Stream::Printer _printer;
};

} // namespace Espfc::Utils
