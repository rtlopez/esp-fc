#pragma once

#include "Stream/Writable.hpp"

// Arduino defines these as macros, keep call sites like print(v, HEX) working with and without it.
#ifndef DEC
#define DEC 10
#endif
#ifndef HEX
#define HEX 16
#endif
#ifndef OCT
#define OCT 8
#endif
#ifndef BIN
#define BIN 2
#endif

namespace Espfc::Stream {

// Text formatter on top of a byte sink. Deliberately non virtual and non owning, it is meant to be
// created ad hoc around any Writable: Printer p{device}; p.println("hello");
// Output is byte compatible with Arduino Print, including println() emitting CRLF, negative values
// printed as two's complement in bases other than 10 and base 0 writing a raw byte.
class Printer
{
public:
  explicit Printer(Writable& out): _out(out) {}

  size_t write(uint8_t c)
  {
    return _out.write(c);
  }
  size_t write(const uint8_t* data, size_t len)
  {
    return _out.write(data, len);
  }
  size_t write(const char* s);

  size_t print(const char* s)
  {
    return write(s);
  }
  size_t print(char c)
  {
    return write(static_cast<uint8_t>(c));
  }
  size_t print(unsigned char v, int base = DEC)
  {
    return printNumber(v, base, sizeof(v));
  }
  size_t print(int v, int base = DEC)
  {
    return printSigned(v, base, sizeof(v));
  }
  size_t print(unsigned int v, int base = DEC)
  {
    return printNumber(v, base, sizeof(v));
  }
  size_t print(long v, int base = DEC)
  {
    return printSigned(v, base, sizeof(v));
  }
  size_t print(unsigned long v, int base = DEC)
  {
    return printNumber(v, base, sizeof(v));
  }
  size_t print(long long v, int base = DEC)
  {
    return printSigned(v, base, sizeof(v));
  }
  size_t print(unsigned long long v, int base = DEC)
  {
    return printNumber(v, base, sizeof(v));
  }
  size_t print(double v, int digits = 2)
  {
    return printFloat(v, digits);
  }

  size_t println();
  size_t println(const char* s)
  {
    return print(s) + println();
  }
  size_t println(char c)
  {
    return print(c) + println();
  }
  size_t println(unsigned char v, int base = DEC)
  {
    return print(v, base) + println();
  }
  size_t println(int v, int base = DEC)
  {
    return print(v, base) + println();
  }
  size_t println(unsigned int v, int base = DEC)
  {
    return print(v, base) + println();
  }
  size_t println(long v, int base = DEC)
  {
    return print(v, base) + println();
  }
  size_t println(unsigned long v, int base = DEC)
  {
    return print(v, base) + println();
  }
  size_t println(long long v, int base = DEC)
  {
    return print(v, base) + println();
  }
  size_t println(unsigned long long v, int base = DEC)
  {
    return print(v, base) + println();
  }
  size_t println(double v, int digits = 2)
  {
    return print(v, digits) + println();
  }

  size_t printf(const char* fmt, ...) __attribute__((format(printf, 2, 3)));

  Writable& out() const
  {
    return _out;
  }

  static constexpr size_t PRINTF_BUFF_SIZE = 128;

private:
  // bytes is the width of the original argument, negative values are printed as two's complement of
  // that width in bases other than 10, exactly like Arduino Print does
  size_t printNumber(unsigned long long v, int base, size_t bytes);
  size_t printSigned(long long v, int base, size_t bytes);
  size_t printFloat(double v, int digits);

  Writable& _out;
};

} // namespace Espfc::Stream
