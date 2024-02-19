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

void test_esc_gcr_to_raw_value()
{
  TEST_ASSERT_EQUAL_UINT32(1942, EscDriver::gcrToRawValue(0b011010011101101100101));
  TEST_ASSERT_EQUAL_UINT32(1944, EscDriver::gcrToRawValue(0b011010011101001110001));
  TEST_ASSERT_EQUAL_UINT32(1947, EscDriver::gcrToRawValue(0b011010011100110110011));
  TEST_ASSERT_EQUAL_UINT32(1948, EscDriver::gcrToRawValue(0b011010011101010001001));
  TEST_ASSERT_EQUAL_UINT32(1949, EscDriver::gcrToRawValue(0b011010011100100101011));
  TEST_ASSERT_EQUAL_UINT32(1950, EscDriver::gcrToRawValue(0b011010011100101110101));
}

void test_esc_gcr_convert_to_value()
{
  TEST_ASSERT_EQUAL_UINT32(3248, EscDriver::convertToValue(1942));
  TEST_ASSERT_EQUAL_UINT32(3264, EscDriver::convertToValue(1944));
  TEST_ASSERT_EQUAL_UINT32(3288, EscDriver::convertToValue(1947));
  TEST_ASSERT_EQUAL_UINT32(3296, EscDriver::convertToValue(1948));
  TEST_ASSERT_EQUAL_UINT32(3304, EscDriver::convertToValue(1949));
  TEST_ASSERT_EQUAL_UINT32(3312, EscDriver::convertToValue(1950));
}

void test_esc_gcr_convert_to_erpm()
{
  TEST_ASSERT_EQUAL_UINT32(185, EscDriver::convertToErpm(EscDriver::convertToValue(1942)));
  TEST_ASSERT_EQUAL_UINT32(184, EscDriver::convertToErpm(EscDriver::convertToValue(1944)));
  TEST_ASSERT_EQUAL_UINT32(182, EscDriver::convertToErpm(EscDriver::convertToValue(1947)));
  TEST_ASSERT_EQUAL_UINT32(182, EscDriver::convertToErpm(EscDriver::convertToValue(1948)));
  TEST_ASSERT_EQUAL_UINT32(182, EscDriver::convertToErpm(EscDriver::convertToValue(1949)));
  TEST_ASSERT_EQUAL_UINT32(181, EscDriver::convertToErpm(EscDriver::convertToValue(1950)));
}

void test_esc_duration_to_bitlen()
{
  uint32_t bit_len = 178;
  TEST_ASSERT_EQUAL_UINT32(0, EscDriver::durationToBitLen(80, bit_len));

  TEST_ASSERT_EQUAL_UINT32(1, EscDriver::durationToBitLen(150, bit_len));
  TEST_ASSERT_EQUAL_UINT32(1, EscDriver::durationToBitLen(160, bit_len));
  TEST_ASSERT_EQUAL_UINT32(1, EscDriver::durationToBitLen(177, bit_len));

  TEST_ASSERT_EQUAL_UINT32(2, EscDriver::durationToBitLen(300, bit_len));
  TEST_ASSERT_EQUAL_UINT32(2, EscDriver::durationToBitLen(373, bit_len));
  TEST_ASSERT_EQUAL_UINT32(2, EscDriver::durationToBitLen(383, bit_len));

  TEST_ASSERT_EQUAL_UINT32(3, EscDriver::durationToBitLen(450, bit_len));
  TEST_ASSERT_EQUAL_UINT32(3, EscDriver::durationToBitLen(498, bit_len));
  TEST_ASSERT_EQUAL_UINT32(3, EscDriver::durationToBitLen(595, bit_len));
  TEST_ASSERT_EQUAL_UINT32(3, EscDriver::durationToBitLen(607, bit_len));
  TEST_ASSERT_EQUAL_UINT32(3, EscDriver::durationToBitLen(610, bit_len));

  TEST_ASSERT_EQUAL_UINT32(4, EscDriver::durationToBitLen(664, bit_len));
}

constexpr uint32_t make_item(uint32_t duration0, uint32_t level0, uint32_t duration1, uint32_t level1)
{
  return (duration0 & 0x07fff) | (level0 & 0x1) << 15 | (duration1 & 0x07fff) << 16 | (level1 & 0x1) << 31;
}

void test_esc_extract_telemetry_gcr_real()
{
  // 0b     0    11     0     1    00   111     0    11     0    11    00     1     0   1
  // 24 0:100 1:200 0:100 1:100 0:200 1:300 0:100 1:200 0:100 1:200 0:200 1:100 0:100
  uint32_t data[] = {
    make_item(100, 0, 200, 1), make_item(100, 0, 100, 1), make_item(200, 0, 300, 1),
    make_item(100, 0, 200, 1), make_item(100, 0, 200, 1), make_item(200, 0, 100, 1),
    make_item(100, 0,   0, 0),
  };
  uint32_t bit_len = 100;
  uint32_t data_len = sizeof(data);
  TEST_ASSERT_EQUAL_UINT32(100, bit_len);
  TEST_ASSERT_EQUAL_UINT32(28, data_len);
  uint32_t exp = 0b011010011101101100101;
  TEST_ASSERT_EQUAL_HEX32(0xD3B65, exp);
  TEST_ASSERT_EQUAL_HEX32(exp, EscDriver::extractTelemetryGcr(data, data_len, bit_len));
}


void test_esc_extract_telemetry_gcr1()
{
  // 24 0:373 1:605 0:163 1:605 0:174 1:376 0:379 1:599 0:177 1:163 0:173 1EF67A 1EF67A?
  //       00   111     0   111     0    11    00   111     0     1     0
  uint32_t data[] = {
    make_item(373, 0, 605, 1), make_item(163, 0, 605, 1), make_item(174, 0, 376, 1),
    make_item(379, 0, 599, 1), make_item(177, 0, 163, 1), make_item(173, 0,   0, 1),
  };
  uint32_t bit_len = 2136 / (1000 / 80);
  uint32_t data_len = sizeof(data);
  TEST_ASSERT_EQUAL_UINT32(178, bit_len);
  TEST_ASSERT_EQUAL_UINT32(24, data_len);
  uint32_t exp = 0b001110111011001110101; // +1
  TEST_ASSERT_EQUAL_HEX32(0x77675, exp);
  TEST_ASSERT_EQUAL_HEX32(exp, EscDriver::extractTelemetryGcr(data, data_len, bit_len));
}

void test_esc_extract_telemetry_gcr2()
{
  // 28 0:177 1:379 0:177 1:167 0:173 1:164 0:379 1:177 0:373 1:379 0:380 1:608 0:164 D499E D499E?
  //        0    11     0     1     0     1    00     1    00    11    00   111     0
  uint32_t data[] = {
    make_item(177, 0, 379, 1), make_item(177, 0, 167, 1), make_item(173, 0, 164, 1),
    make_item(379, 0, 177, 1), make_item(373, 0, 379, 1), make_item(380, 0, 608, 1),
    make_item(164, 0, 0, 0),
  };
  uint32_t bit_len = 2136 / (1000 / 80);
  uint32_t data_len = sizeof(data);
  uint32_t exp = 0b011010100100110011101; // +1
  TEST_ASSERT_EQUAL_UINT32(178, bit_len);
  TEST_ASSERT_EQUAL_UINT32(28, data_len);
  TEST_ASSERT_EQUAL_HEX32(0xD499D, exp);
  TEST_ASSERT_EQUAL_HEX32(exp, EscDriver::extractTelemetryGcr(data, data_len, bit_len));
}

int main(int argc, char **argv)
{
  UNITY_BEGIN();
  RUN_TEST(test_esc_dshot_encode);
  RUN_TEST(test_esc_dshot_encode_inverted);
  RUN_TEST(test_esc_dshot_convert);
  RUN_TEST(test_esc_gcr_to_raw_value);
  RUN_TEST(test_esc_gcr_convert_to_value);
  RUN_TEST(test_esc_gcr_convert_to_erpm);
  RUN_TEST(test_esc_duration_to_bitlen);
  RUN_TEST(test_esc_extract_telemetry_gcr_real);
  RUN_TEST(test_esc_extract_telemetry_gcr1);
  RUN_TEST(test_esc_extract_telemetry_gcr2);
  UNITY_END();

  return 0;
}
