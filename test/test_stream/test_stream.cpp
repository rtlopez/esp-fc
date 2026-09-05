#include <unity.h>

// not used directly, but required so that the library dependency finder gives lib/betaflight
// the Espfc include path, same reason as in test_msp
#include <platform.h>

#include <Hal/Serial.hpp>
#include <Stream/BufferWritable.hpp>
#include <Stream/Printer.hpp>
#include <Stream/ReadWritable.hpp>

#include <cmath>
#include <cstring>

using namespace Espfc::Stream;

static char buff[256];
static BufferWritable sink{buff, sizeof(buff)};
static Printer printer{sink};

void setUp(void)
{
  sink.clear();
}

void tearDown(void) {}

// buffer sink

void test_buffer_write_byte()
{
  TEST_ASSERT_EQUAL(1, sink.write('a'));
  TEST_ASSERT_EQUAL(1, sink.size());
  TEST_ASSERT_EQUAL_STRING("a", sink.c_str());
  TEST_ASSERT_FALSE(sink.overflow());
}

void test_buffer_write_many()
{
  const char* text = "hello";
  TEST_ASSERT_EQUAL(5, sink.write(reinterpret_cast<const uint8_t*>(text), 5));
  TEST_ASSERT_EQUAL_STRING("hello", sink.c_str());
  TEST_ASSERT_EQUAL(sizeof(buff) - 1 - 5, sink.availableForWrite());
}

void test_buffer_overflow()
{
  char small[4];
  BufferWritable s{small, sizeof(small)};

  TEST_ASSERT_EQUAL(3, s.availableForWrite());
  TEST_ASSERT_EQUAL(3, s.write(reinterpret_cast<const uint8_t*>("abcdef"), 6));
  TEST_ASSERT_EQUAL_STRING("abc", s.c_str());
  TEST_ASSERT_TRUE(s.overflow());
  TEST_ASSERT_EQUAL(0, s.availableForWrite());
  TEST_ASSERT_EQUAL(0, s.write('x'));

  s.clear();
  TEST_ASSERT_EQUAL_STRING("", s.c_str());
  TEST_ASSERT_FALSE(s.overflow());
}

// raw writes and strings

void test_printer_write()
{
  TEST_ASSERT_EQUAL(1, printer.write(static_cast<uint8_t>(0x02)));
  TEST_ASSERT_EQUAL(1, sink.size());
  TEST_ASSERT_EQUAL(0x02, sink.c_str()[0]);
}

void test_printer_print_string()
{
  TEST_ASSERT_EQUAL(3, printer.print("abc"));
  TEST_ASSERT_EQUAL(0, printer.print(static_cast<const char*>(nullptr)));
  TEST_ASSERT_EQUAL_STRING("abc", sink.c_str());
}

void test_printer_print_char()
{
  // char must be emitted verbatim, not as a number
  printer.print('S');
  printer.print(' ');
  printer.print('N');
  TEST_ASSERT_EQUAL_STRING("S N", sink.c_str());
}

void test_printer_print_bool()
{
  // there is no bool overload on purpose, bool promotes to int and prints as 1/0
  printer.print(true);
  printer.print(false);
  TEST_ASSERT_EQUAL_STRING("10", sink.c_str());
}

void test_printer_print_uchar_is_numeric()
{
  const uint8_t v = 65;
  printer.print(v);
  TEST_ASSERT_EQUAL_STRING("65", sink.c_str());
}

// integers, base 10

void test_printer_print_int()
{
  printer.print(0);
  printer.print(' ');
  printer.print(42);
  printer.print(' ');
  printer.print(-42);
  TEST_ASSERT_EQUAL_STRING("0 42 -42", sink.c_str());
}

void test_printer_print_int_limits()
{
  printer.print(static_cast<int32_t>(2147483647));
  printer.print(' ');
  printer.print(static_cast<int32_t>(-2147483647 - 1));
  TEST_ASSERT_EQUAL_STRING("2147483647 -2147483648", sink.c_str());
}

void test_printer_print_unsigned_limits()
{
  printer.print(static_cast<uint32_t>(4294967295u));
  TEST_ASSERT_EQUAL_STRING("4294967295", sink.c_str());
}

void test_printer_print_long_long()
{
  printer.print(static_cast<long long>(-9007199254740993ll));
  TEST_ASSERT_EQUAL_STRING("-9007199254740993", sink.c_str());
}

// bases

void test_printer_print_hex()
{
  printer.print(static_cast<uint8_t>(0x0a), HEX);
  printer.print(' ');
  printer.print(static_cast<uint32_t>(0xdeadbeef), HEX);
  TEST_ASSERT_EQUAL_STRING("A DEADBEEF", sink.c_str());
}

void test_printer_print_bin_oct()
{
  printer.print(static_cast<uint8_t>(5), BIN);
  printer.print(' ');
  printer.print(static_cast<uint8_t>(64), OCT);
  TEST_ASSERT_EQUAL_STRING("101 100", sink.c_str());
}

void test_printer_print_negative_hex_masked()
{
  // negative values are two's complement of the argument width, same on 32 and 64 bit hosts
  printer.print(static_cast<int>(-1), HEX);
  printer.print(' ');
  printer.print(static_cast<long long>(-1), HEX);
  TEST_ASSERT_EQUAL_STRING("FFFFFFFF FFFFFFFFFFFFFFFF", sink.c_str());
}

void test_printer_print_base_zero_writes_raw()
{
  printer.print(static_cast<uint8_t>(0x41), 0);
  TEST_ASSERT_EQUAL_STRING("A", sink.c_str());
}

void test_printer_print_base_one_falls_back_to_dec()
{
  printer.print(static_cast<uint8_t>(12), 1);
  TEST_ASSERT_EQUAL_STRING("12", sink.c_str());
}

// floats

void test_printer_print_float_default_digits()
{
  printer.print(1.5f);
  TEST_ASSERT_EQUAL_STRING("1.50", sink.c_str());
}

void test_printer_print_float_digits()
{
  printer.print(3.14159f, 4);
  TEST_ASSERT_EQUAL_STRING("3.1416", sink.c_str());
}

void test_printer_print_float_zero_digits()
{
  printer.print(2.6f, 0);
  TEST_ASSERT_EQUAL_STRING("3", sink.c_str());
}

void test_printer_print_float_rounding()
{
  printer.print(1.999, 2);
  TEST_ASSERT_EQUAL_STRING("2.00", sink.c_str());
}

void test_printer_print_float_negative()
{
  printer.print(-0.05, 2);
  TEST_ASSERT_EQUAL_STRING("-0.05", sink.c_str());
}

void test_printer_print_float_specials()
{
  printer.print(NAN);
  printer.print(' ');
  printer.print(INFINITY);
  printer.print(' ');
  printer.print(5e9);
  printer.print(' ');
  printer.print(-5e9);
  TEST_ASSERT_EQUAL_STRING("nan inf ovf ovf", sink.c_str());
}

// println and printf

void test_printer_println_is_crlf()
{
  TEST_ASSERT_EQUAL(2, printer.println());
  TEST_ASSERT_EQUAL_STRING("\r\n", sink.c_str());
}

void test_printer_println_value()
{
  printer.println("ok");
  printer.println(7);
  TEST_ASSERT_EQUAL_STRING("ok\r\n7\r\n", sink.c_str());
}

void test_printer_printf()
{
  printer.printf("|  %02x  | 0x%06X | %-4s |\r\n", 5, 0x1234, "ab");
  TEST_ASSERT_EQUAL_STRING("|  05  | 0x001234 | ab   |\r\n", sink.c_str());
}

void test_printer_printf_size_t()
{
  const size_t v = 1234;
  printer.printf("total: %zu", v);
  TEST_ASSERT_EQUAL_STRING("total: 1234", sink.c_str());
}

void test_printer_printf_truncates()
{
  char big[Printer::PRINTF_BUFF_SIZE * 2];
  std::memset(big, 'x', sizeof(big) - 1);
  big[sizeof(big) - 1] = '\0';

  const size_t written = printer.printf("%s", big);
  TEST_ASSERT_EQUAL(Printer::PRINTF_BUFF_SIZE - 1, written);
  TEST_ASSERT_EQUAL(Printer::PRINTF_BUFF_SIZE - 1, sink.size());
}

// interfaces

class TestDevice : public ReadWritable
{
public:
  size_t write(uint8_t c) override
  {
    _last = c;
    return 1;
  }
  size_t write(const uint8_t* data, size_t len) override
  {
    if (len) _last = data[len - 1];
    return len;
  }
  int availableForWrite() override
  {
    return 1;
  }
  void flush() override {}
  int available() override
  {
    return 1;
  }
  int read() override
  {
    return _last;
  }
  size_t readMany(uint8_t* data, size_t len) override
  {
    if (len) data[0] = _last;
    return len ? 1 : 0;
  }
  int peek() override
  {
    return _last;
  }
  bool isTxFifoEmpty() override
  {
    return true;
  }
  void begin(const Espfc::Hal::SerialDeviceConfig& config) override {}
  void updateBaudRate(int baud) override {}

  uint8_t _last = 0;
};

void test_readwritable_is_usable_by_printer()
{
  TestDevice dev;
  Printer p{dev};

  p.print(123);
  TEST_ASSERT_EQUAL('3', dev.read());
  TEST_ASSERT_EQUAL_PTR(static_cast<Writable*>(&dev), &p.out());
}

int main(int argc, char** argv)
{
  UNITY_BEGIN();

  RUN_TEST(test_buffer_write_byte);
  RUN_TEST(test_buffer_write_many);
  RUN_TEST(test_buffer_overflow);

  RUN_TEST(test_printer_write);
  RUN_TEST(test_printer_print_string);
  RUN_TEST(test_printer_print_char);
  RUN_TEST(test_printer_print_bool);
  RUN_TEST(test_printer_print_uchar_is_numeric);

  RUN_TEST(test_printer_print_int);
  RUN_TEST(test_printer_print_int_limits);
  RUN_TEST(test_printer_print_unsigned_limits);
  RUN_TEST(test_printer_print_long_long);

  RUN_TEST(test_printer_print_hex);
  RUN_TEST(test_printer_print_bin_oct);
  RUN_TEST(test_printer_print_negative_hex_masked);
  RUN_TEST(test_printer_print_base_zero_writes_raw);
  RUN_TEST(test_printer_print_base_one_falls_back_to_dec);

  RUN_TEST(test_printer_print_float_default_digits);
  RUN_TEST(test_printer_print_float_digits);
  RUN_TEST(test_printer_print_float_zero_digits);
  RUN_TEST(test_printer_print_float_rounding);
  RUN_TEST(test_printer_print_float_negative);
  RUN_TEST(test_printer_print_float_specials);

  RUN_TEST(test_printer_println_is_crlf);
  RUN_TEST(test_printer_println_value);
  RUN_TEST(test_printer_printf);
  RUN_TEST(test_printer_printf_size_t);
  RUN_TEST(test_printer_printf_truncates);

  RUN_TEST(test_readwritable_is_usable_by_printer);

  return UNITY_END();
}
