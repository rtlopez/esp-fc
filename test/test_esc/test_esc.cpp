#include <unity.h>
#include <ArduinoFake.h>
#include "EscDriver.h"

void test_esc_dshot_encode()
{
  TEST_ASSERT_EQUAL_UINT16(    0, EscDriver::dshotEncode(   0, false));
  TEST_ASSERT_EQUAL_UINT16(   34, EscDriver::dshotEncode(   1, false));
  TEST_ASSERT_EQUAL_UINT16(   68, EscDriver::dshotEncode(   2, false));
  TEST_ASSERT_EQUAL_UINT16( 1515, EscDriver::dshotEncode(  47, false));
  TEST_ASSERT_EQUAL_UINT16(32010, EscDriver::dshotEncode(1000, false));
  TEST_ASSERT_EQUAL_UINT16(32040, EscDriver::dshotEncode(1001, false));
  TEST_ASSERT_EQUAL_UINT16(32742, EscDriver::dshotEncode(1023, false));
  TEST_ASSERT_EQUAL_UINT16(32776, EscDriver::dshotEncode(1024, false));
  TEST_ASSERT_EQUAL_UINT16(65518, EscDriver::dshotEncode(2047, false));
}

void test_esc_dshot_encode_inverted()
{
  TEST_ASSERT_EQUAL_UINT16(   15, EscDriver::dshotEncode(0,    true));
  TEST_ASSERT_EQUAL_UINT16(   45, EscDriver::dshotEncode(1,    true));
  TEST_ASSERT_EQUAL_UINT16(   75, EscDriver::dshotEncode(2,    true));
  TEST_ASSERT_EQUAL_UINT16( 1508, EscDriver::dshotEncode(47,   true));
  TEST_ASSERT_EQUAL_UINT16(32005, EscDriver::dshotEncode(1000, true));
  TEST_ASSERT_EQUAL_UINT16(32039, EscDriver::dshotEncode(1001, true));
  TEST_ASSERT_EQUAL_UINT16(32745, EscDriver::dshotEncode(1023, true));
  TEST_ASSERT_EQUAL_UINT16(32775, EscDriver::dshotEncode(1024, true));
  TEST_ASSERT_EQUAL_UINT16(65505, EscDriver::dshotEncode(2047, true));
}

void test_esc_dshot_convert()
{
  TEST_ASSERT_EQUAL_UINT16(   0, EscDriver::dshotConvert(   0));
  TEST_ASSERT_EQUAL_UINT16(   0, EscDriver::dshotConvert(   1));
  TEST_ASSERT_EQUAL_UINT16(   0, EscDriver::dshotConvert( 999));
  TEST_ASSERT_EQUAL_UINT16(   0, EscDriver::dshotConvert(1000));
  TEST_ASSERT_EQUAL_UINT16(  49, EscDriver::dshotConvert(1001));
  TEST_ASSERT_EQUAL_UINT16(1047, EscDriver::dshotConvert(1500));
  TEST_ASSERT_EQUAL_UINT16(2047, EscDriver::dshotConvert(2000));
}

int main(int argc, char **argv)
{
  UNITY_BEGIN();
  RUN_TEST(test_esc_dshot_encode);
  RUN_TEST(test_esc_dshot_encode_inverted);
  RUN_TEST(test_esc_dshot_convert);
  UNITY_END();

  return 0;
}
