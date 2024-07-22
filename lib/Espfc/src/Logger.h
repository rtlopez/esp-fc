#ifndef _ESPFC_LOGGER_H_
#define _ESPFC_LOGGER_H_

#include <Arduino.h>
#include "Debug_Espfc.h"

namespace Espfc {

class Logger
{
  public:
    int begin(size_t size = 1024)
    {
      _size = size;
      _tail = 0;
      _buff = new char[size];
      _buff[0] = '\0';
      return 1;
    }

    Logger& info()
    {
      LOG_SERIAL_DEBUG("I")
      append('I');
      log(String{(float)millis() * 0.001f, 2});
      return *this;
    }

    Logger& err()
    {
      LOG_SERIAL_DEBUG("E")
      append('E');
      log(String{(float)millis() * 0.001f, 2});
      return *this;
    }

    template<typename T>
    Logger& log(const T& v)
    {
      LOG_SERIAL_DEBUG(' ')
      LOG_SERIAL_DEBUG(v)
      append(' ');
      append(String(v));
      return *this;
    }

    template<typename T>
    Logger& loghex(const T& v)
    {
      LOG_SERIAL_DEBUG(' ')
      LOG_SERIAL_DEBUG_HEX(v)
      append(' ');
      append(String(v, HEX));
      return *this;
    }

    template<typename T>
    Logger& logln(const T& v)
    {
      LOG_SERIAL_DEBUG(' ')
      LOG_SERIAL_DEBUG(v)
      append(' ');
      append(String(v));
      return endl();
    }

    Logger& endl()
    {
      LOG_SERIAL_DEBUG('\r')
      LOG_SERIAL_DEBUG('\n')
      append('\r');
      append('\n');
      return *this;
    }

    void append(const String& s)
    {
      append(s.c_str(), s.length());
    }

    void append(const char * s, size_t len)
    {
      for(size_t i = 0; i < len; i++)
      {
        append(s[i]);
      }
    }

    void append(char c)
    {
      if(_size > 0 && _tail < _size - 1)
      {
        _buff[_tail] = c;
        _tail++;
        _buff[_tail] = '\0';
      }
    }

    const char * c_str() const
    {
      return _buff;
    }

    const size_t length() const
    {
      return _tail;
    }

  private:
    char * _buff;
    size_t _size;
    size_t _tail;
};

}

#endif
