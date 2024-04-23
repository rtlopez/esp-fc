#include <unity.h>
#include <ArduinoFake.h>
#include <EscDriver.h>
#include <helper_3dmath.h>
#include <Kalman.h>
#include "msp/msp_protocol.h"
#include <printf.h>

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
  TEST_ASSERT_EQUAL_UINT32(  60, EscDriver::convertToErpm(10000));
  TEST_ASSERT_EQUAL_UINT32( 120, EscDriver::convertToErpm( 5000));
  TEST_ASSERT_EQUAL_UINT32( 300, EscDriver::convertToErpm( 2000));
  TEST_ASSERT_EQUAL_UINT32( 600, EscDriver::convertToErpm( 1000));
  TEST_ASSERT_EQUAL_UINT32(1200, EscDriver::convertToErpm(  500));
  TEST_ASSERT_EQUAL_UINT32(3000, EscDriver::convertToErpm(  200));
  TEST_ASSERT_EQUAL_UINT32(6000, EscDriver::convertToErpm(  100));
}

void test_esc_duration_to_bitlen()
{
  uint32_t bit_len = 213; // dshot300: 2.666us/12.5, dshot600: 1.333us/12.5
  TEST_ASSERT_EQUAL_UINT32(0, EscDriver::durationToBitLen( 80, bit_len));

  TEST_ASSERT_EQUAL_UINT32(1, EscDriver::durationToBitLen(150, bit_len));
  TEST_ASSERT_EQUAL_UINT32(1, EscDriver::durationToBitLen(160, bit_len));
  TEST_ASSERT_EQUAL_UINT32(1, EscDriver::durationToBitLen(177, bit_len));

  TEST_ASSERT_EQUAL_UINT32(2, EscDriver::durationToBitLen(373, bit_len));
  TEST_ASSERT_EQUAL_UINT32(2, EscDriver::durationToBitLen(383, bit_len));

  TEST_ASSERT_EQUAL_UINT32(3, EscDriver::durationToBitLen(595, bit_len));
  TEST_ASSERT_EQUAL_UINT32(3, EscDriver::durationToBitLen(607, bit_len));
  TEST_ASSERT_EQUAL_UINT32(3, EscDriver::durationToBitLen(610, bit_len));
  TEST_ASSERT_EQUAL_UINT32(3, EscDriver::durationToBitLen(640, bit_len));

  TEST_ASSERT_EQUAL_UINT32(4, EscDriver::durationToBitLen(850, bit_len));
}

constexpr uint32_t make_item(uint32_t duration0, uint32_t level0, uint32_t duration1, uint32_t level1)
{
  return (duration0 & 0x07fff) | (level0 & 0x1) << 15 | (duration1 & 0x07fff) << 16 | (level1 & 0x1) << 31;
}

void test_esc_extract_telemetry_gcr_synth()
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

void test_esc_extract_telemetry_gcr_synth_idle()
{
  // 0b    00     1     0     1    00     1     0     1    00     1     0     1     0     1   000  1
  // 32 0:200 1:100 0:100 1:100 0:200 1:100 0:100 1:100 0:200 1:100 0:100 1:100 0:100 1:100 0:300
  uint32_t data[] = {
    make_item(200, 0, 100, 1), make_item(100, 0, 100, 1), make_item(200, 0, 100, 1),
    make_item(100, 0, 100, 1), make_item(200, 0, 100, 1), make_item(100, 0, 100, 1),
    make_item(100, 0, 100, 1), make_item(300, 0,   0, 1),
  };
  uint32_t bit_len = 100;
  uint32_t data_len = sizeof(data);
  TEST_ASSERT_EQUAL_UINT32(100, bit_len);
  TEST_ASSERT_EQUAL_UINT32(32, data_len);
  uint32_t gcr = 0b001010010100101010001;
  TEST_ASSERT_EQUAL_HEX32(0x52951, gcr);
  TEST_ASSERT_EQUAL_HEX32(gcr, EscDriver::extractTelemetryGcr(data, data_len, bit_len));

  uint32_t value = EscDriver::gcrToRawValue(gcr);
  TEST_ASSERT_EQUAL_HEX32(0x0fff, value);

  value = EscDriver::convertToValue(value);
  TEST_ASSERT_EQUAL_HEX32(0x0, value);

  uint32_t erpm = EscDriver::convertToErpm(value);
  TEST_ASSERT_EQUAL_UINT32(0, erpm);

  float erpmToHz = EscDriver::getErpmToHzRatio(14);
  float freq = erpmToHz * erpm;
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, freq);
  float rpm = freq * 60;
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, rpm);
}

void test_esc_extract_telemetry_dshot300_sample()
{
  // 24 0:373 1:605 0:163 1:605 0:174 1:376 0:379 1:599 0:177 1:163 0:173 1EF67A 1EF67A?
  //       00   111     0   111     0    11    00   111     0     1     0
  uint32_t data[] = {
    make_item(373, 0, 605, 1), make_item(163, 0, 605, 1), make_item(174, 0, 376, 1),
    make_item(379, 0, 599, 1), make_item(177, 0, 163, 1), make_item(173, 0,   0, 1),
  };
  uint32_t bit_len = 2667 / (1000.0 / 80);
  uint32_t data_len = sizeof(data);
  TEST_ASSERT_EQUAL_UINT32(213, bit_len);
  TEST_ASSERT_EQUAL_UINT32(24, data_len);
  uint32_t gcr = 0b001110111011001110101; // +1
  TEST_ASSERT_EQUAL_HEX32(0x77675, gcr);
  TEST_ASSERT_EQUAL_HEX32(gcr, EscDriver::extractTelemetryGcr(data, data_len, bit_len));

  uint32_t value = EscDriver::gcrToRawValue(gcr);
  TEST_ASSERT_EQUAL_HEX32(0x093a, value);

  value = EscDriver::convertToValue(value);
  TEST_ASSERT_EQUAL_HEX32(0x13a0, value);

  uint32_t erpm = EscDriver::convertToErpm(value);
  TEST_ASSERT_EQUAL_UINT32(119, erpm);

  float erpmToHz = EscDriver::getErpmToHzRatio(14);
  float freq = erpmToHz * erpm;
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 28.33f, freq);
  float rpm = freq * 60;
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 1700.0f, rpm);
}

void test_esc_extract_telemetry_dshot300_running()
{
  // 28 0:177 1:379 0:177 1:167 0:173 1:164 0:379 1:177 0:373 1:379 0:380 1:608 0:164 D499E D499E?
  //        0    11     0     1     0     1    00     1    00    11    00   111     0
  uint32_t data[] = {
    make_item(177, 0, 379, 1), make_item(177, 0, 167, 1), make_item(173, 0, 164, 1),
    make_item(379, 0, 177, 1), make_item(373, 0, 379, 1), make_item(380, 0, 608, 1),
    make_item(164, 0, 0, 0),
  };
  uint32_t bit_len = 2667 / (1000.0 / 80);
  uint32_t data_len = sizeof(data);
  uint32_t gcr = 0b011010100100110011101; // +1
  TEST_ASSERT_EQUAL_UINT32(213, bit_len);
  TEST_ASSERT_EQUAL_UINT32(28, data_len);
  TEST_ASSERT_EQUAL_HEX32(0xD499D, gcr);
  TEST_ASSERT_EQUAL_HEX32(gcr, EscDriver::extractTelemetryGcr(data, data_len, bit_len));

  uint32_t value = EscDriver::gcrToRawValue(gcr);
  TEST_ASSERT_EQUAL_HEX32(0x071a, value);

  value = EscDriver::convertToValue(value);
  TEST_ASSERT_EQUAL_HEX32(0x08d0, value);

  uint32_t erpm = EscDriver::convertToErpm(value);
  TEST_ASSERT_EQUAL_UINT32(266, erpm);

  float erpmToHz = EscDriver::getErpmToHzRatio(14);
  float freq = erpmToHz * erpm;
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 63.33f, freq);
  float rpm = freq * 60;
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 3800.0f, rpm);
}

void test_esc_extract_telemetry_dshot300_idle()
{
  // 32 0:380 1:172 0:175 1:172 0:371 1:172 0:175 1:172 0:371 1:172 0:174 1:162 0:172 1:172 0:606 52950 52950
  //       00     1     0     1    00     1     0     1    00     1     0     1     0     1   000  1
  uint32_t data[] = {
    make_item(380, 0, 172, 1), make_item(175, 0, 172, 1), make_item(371, 0, 172, 1), 
    make_item(175, 0, 172, 1), make_item(371, 0, 172, 1), make_item(174, 0, 162, 1),
    make_item(172, 0, 172, 1), make_item(606, 0, 0, 1),
  };
  uint32_t bit_len = 2667 / (1000.0 / 80);
  uint32_t data_len = sizeof(data);
  TEST_ASSERT_EQUAL_UINT32(213, bit_len);
  TEST_ASSERT_EQUAL_UINT32(32, data_len);
  uint32_t gcr = 0b001010010100101010001; // +1
  TEST_ASSERT_EQUAL_HEX32(0x52951, gcr);
  TEST_ASSERT_EQUAL_HEX32(gcr, EscDriver::extractTelemetryGcr(data, data_len, bit_len));

  uint32_t value = EscDriver::gcrToRawValue(gcr);
  TEST_ASSERT_EQUAL_HEX32(0x0fff, value);

  value = EscDriver::convertToValue(value);
  TEST_ASSERT_EQUAL_HEX32(0x0, value);

  uint32_t erpm = EscDriver::convertToErpm(value);
  TEST_ASSERT_EQUAL_UINT32(0, erpm);

  float erpmToHz = EscDriver::getErpmToHzRatio(14);
  float freq = erpmToHz * erpm;
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, freq);
  float rpm = freq * 60;
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, rpm);
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
  RUN_TEST(test_esc_extract_telemetry_gcr_synth);
  RUN_TEST(test_esc_extract_telemetry_gcr_synth_idle);
  RUN_TEST(test_esc_extract_telemetry_dshot300_sample);
  RUN_TEST(test_esc_extract_telemetry_dshot300_running);
  RUN_TEST(test_esc_extract_telemetry_dshot300_idle);
  UNITY_END();

  return 0;
}
