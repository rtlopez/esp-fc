#include "Stream/Printer.hpp"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>

namespace Espfc::Stream {

namespace {

// Renders v into the tail of buff and returns a pointer to the first digit.
template<typename T>
const char* renderNumber(T v, int base, char* buff, size_t size)
{
  if (base < 2) base = 10;
  char* str = buff + size - 1;
  *str = '\0';
  do
  {
    const int digit = static_cast<int>(v % static_cast<T>(base));
    v /= static_cast<T>(base);
    *--str = digit < 10 ? static_cast<char>('0' + digit) : static_cast<char>('A' + digit - 10);
  } while (v);
  return str;
}

} // namespace

size_t Printer::write(const char* s)
{
  if (!s) return 0;
  return write(reinterpret_cast<const uint8_t*>(s), std::strlen(s));
}

size_t Printer::println()
{
  return write("\r\n");
}

size_t Printer::printNumber(unsigned long long v, int base, size_t bytes)
{
  if (base == 0) return write(static_cast<uint8_t>(v));

  if (bytes < sizeof(unsigned long long))
  {
    v &= (1ull << (8 * bytes)) - 1ull;
  }

  if (bytes <= sizeof(uint32_t))
  {
    char buff[8 * sizeof(uint32_t) + 1];
    return write(renderNumber(static_cast<uint32_t>(v), base, buff, sizeof(buff)));
  }

  char buff[8 * sizeof(unsigned long long) + 1];
  return write(renderNumber(v, base, buff, sizeof(buff)));
}

size_t Printer::printSigned(long long v, int base, size_t bytes)
{
  if (base == 0) return write(static_cast<uint8_t>(v));
  if (base != 10) return printNumber(static_cast<unsigned long long>(v), base, bytes);
  if (v < 0)
  {
    // negate as unsigned, so that the most negative value does not overflow
    return print('-') + printNumber(-static_cast<unsigned long long>(v), 10, sizeof(v));
  }
  return printNumber(static_cast<unsigned long long>(v), 10, sizeof(v));
}

size_t Printer::printFloat(double v, int digits)
{
  if (std::isnan(v)) return print("nan");
  if (std::isinf(v)) return print("inf");
  if (v > 4294967040.0 || v < -4294967040.0) return print("ovf");

  digits = std::clamp(digits, 0, 17);

  size_t n = 0;
  if (v < 0.0)
  {
    n += print('-');
    v = -v;
  }

  // round, so that print(1.999, 2) yields 2.00
  double rounding = 0.5;
  for (int i = 0; i < digits; i++)
  {
    rounding *= 0.1;
  }
  v += rounding;

  const unsigned long intPart = static_cast<unsigned long>(v);
  double remainder = v - static_cast<double>(intPart);
  n += printNumber(intPart, 10, sizeof(intPart));

  if (digits > 0) n += print('.');

  while (digits-- > 0)
  {
    remainder *= 10.0;
    const unsigned int digit = static_cast<unsigned int>(remainder);
    n += printNumber(digit, 10, sizeof(digit));
    remainder -= digit;
  }

  return n;
}

size_t Printer::printf(const char* fmt, ...)
{
  char buff[PRINTF_BUFF_SIZE];

  va_list ap;
  va_start(ap, fmt);
  const int len = vsnprintf(buff, sizeof(buff), fmt, ap);
  va_end(ap);

  if (len <= 0) return 0;

  // output is truncated to the buffer size, no dynamic allocation on purpose
  return write(reinterpret_cast<const uint8_t*>(buff), std::min(static_cast<size_t>(len), sizeof(buff) - 1));
}

} // namespace Espfc::Stream
