#include "Utils/Logger.hpp"

namespace Espfc::Utils {

Logger::Logger(): _buff(nullptr), _printer(_sink) {}

Logger::~Logger()
{
  _sink.reset(nullptr, 0);
  delete[] _buff;
  _buff = nullptr;
}

int Logger::begin(size_t size)
{
  _sink.reset(nullptr, 0);
  delete[] _buff;
  _buff = new char[size];
  _sink.reset(_buff, size);
  return 1;
}

Logger& Logger::info()
{
  LOG_SERIAL_DEBUG("I")
  _printer.print('I');
  return log((float)millis() * 0.001f);
}

Logger& Logger::err()
{
  LOG_SERIAL_DEBUG("E")
  _printer.print('E');
  return log((float)millis() * 0.001f);
}

Logger& Logger::endl()
{
  LOG_SERIAL_DEBUG('\r')
  LOG_SERIAL_DEBUG('\n')
  _printer.println();
  return *this;
}

const char* Logger::c_str() const
{
  return _sink.c_str();
}

size_t Logger::length() const
{
  return _sink.size();
}

} // namespace Espfc::Utils
