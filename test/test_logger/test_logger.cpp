#include <ArduinoFake.h>
#include <unity.h>

// not used directly, but required so that the library dependency finder gives lib/betaflight
// the Espfc include path, same reason as in test_msp
#include <platform.h>

#include <Utils/Logger.hpp>

using Logger = Espfc::Utils::Logger;
using namespace fakeit;

void setUp(void)
{
  ArduinoFakeReset();
  When(Method(ArduinoFake(Function), millis)).AlwaysReturn(1500);
}

void tearDown(void) {}

void test_logger_empty_before_begin()
{
  Logger logger;
  TEST_ASSERT_NULL(logger.c_str());
  TEST_ASSERT_EQUAL(0, logger.length());
}

void test_logger_empty_after_begin()
{
  Logger logger;
  logger.begin(64);
  TEST_ASSERT_EQUAL_STRING("", logger.c_str());
  TEST_ASSERT_EQUAL(0, logger.length());
}

void test_logger_info_timestamp()
{
  Logger logger;
  logger.begin(64);
  logger.info().logln("X");
  TEST_ASSERT_EQUAL_STRING("I 1.50 X\r\n", logger.c_str());
  TEST_ASSERT_EQUAL(10, logger.length());
}

void test_logger_err_mixed_types()
{
  Logger logger;
  logger.begin(64);
  logger.err().log("A").log(42).logln(true);
  TEST_ASSERT_EQUAL_STRING("E 1.50 A 42 1\r\n", logger.c_str());
}

void test_logger_log_float()
{
  Logger logger;
  logger.begin(64);
  logger.log(1.5f);
  TEST_ASSERT_EQUAL_STRING(" 1.50", logger.c_str());
}

void test_logger_log_negative_and_unsigned()
{
  Logger logger;
  logger.begin(64);
  logger.log((int8_t)-7).log((uint8_t)200).logln((uint32_t)4294967295u);
  TEST_ASSERT_EQUAL_STRING(" -7 200 4294967295\r\n", logger.c_str());
}

void test_logger_log_char_array()
{
  Logger logger;
  logger.begin(64);
  char text[8] = "ssid";
  const char* ptr = "ptr";
  logger.log(text).logln(ptr);
  TEST_ASSERT_EQUAL_STRING(" ssid ptr\r\n", logger.c_str());
}

void test_logger_loghex_is_uppercase()
{
  Logger logger;
  logger.begin(64);
  logger.loghex((uint8_t)0xea).loghex((uint8_t)0x0f);
  TEST_ASSERT_EQUAL_STRING(" EA F", logger.c_str());
}

void test_logger_endl()
{
  Logger logger;
  logger.begin(64);
  logger.log("a").endl();
  TEST_ASSERT_EQUAL_STRING(" a\r\n", logger.c_str());
}

void test_logger_truncates_on_overflow()
{
  Logger logger;
  logger.begin(8);
  logger.info().logln("some long text that does not fit");

  TEST_ASSERT_EQUAL(7, logger.length());
  TEST_ASSERT_EQUAL_STRING("I 1.50 ", logger.c_str());
}

void test_logger_begin_twice_resets()
{
  Logger logger;
  logger.begin(64);
  logger.log("first");
  logger.begin(64);
  TEST_ASSERT_EQUAL_STRING("", logger.c_str());
  TEST_ASSERT_EQUAL(0, logger.length());

  logger.log("second");
  TEST_ASSERT_EQUAL_STRING(" second", logger.c_str());
}

int main(int argc, char** argv)
{
  UNITY_BEGIN();

  RUN_TEST(test_logger_empty_before_begin);
  RUN_TEST(test_logger_empty_after_begin);
  RUN_TEST(test_logger_info_timestamp);
  RUN_TEST(test_logger_err_mixed_types);
  RUN_TEST(test_logger_log_float);
  RUN_TEST(test_logger_log_negative_and_unsigned);
  RUN_TEST(test_logger_log_char_array);
  RUN_TEST(test_logger_loghex_is_uppercase);
  RUN_TEST(test_logger_endl);
  RUN_TEST(test_logger_truncates_on_overflow);
  RUN_TEST(test_logger_begin_twice_resets);

  return UNITY_END();
}
