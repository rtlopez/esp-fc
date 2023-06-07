#include <unity.h>
#include <ArduinoFake.h>
#include "Device/InputCRSF.h"
#include <EspGpio.h>

using namespace Espfc;
using namespace Espfc::Device;

void test_input_crsf_rc_valid()
{
  InputCRSF input;
  CrsfFrame frame;
  memset(&frame, 0, sizeof(frame));

  input.begin(nullptr);

  const uint8_t data[] = {
    0xC8, 0x18, 0x16, 0xE0, 0x03, 0xDF, 0xD9, 0xC0, 0xF7, 0x8B, 0x5F, 0x94, 0xAF,
    0x7C, 0xE5, 0x2B, 0x5F, 0xF9, 0xCA, 0x07, 0x00, 0x00, 0x4C, 0x7C, 0xE2, 0x23
  };
  for (size_t i = 0; i < sizeof(data); i++) {
    input.parse(frame, data[i]);
  }

  for (size_t i; i < sizeof(data); i++) {
    TEST_ASSERT_EQUAL_UINT8(data[i], frame.data[i]);
  }

  const uint8_t crc = input.frameCrc(frame);
  TEST_ASSERT_EQUAL_UINT8(0x23, crc);

  TEST_ASSERT_EQUAL_UINT8(CRSF_ADDRESS_FLIGHT_CONTROLLER, frame.message.addr);
  TEST_ASSERT_EQUAL_UINT8(0x18, frame.message.size);
  TEST_ASSERT_EQUAL_UINT8(CRSF_FRAMETYPE_RC_CHANNELS_PACKED, frame.message.type);

  TEST_ASSERT_EQUAL_UINT16(1500u, input.get(0));
  TEST_ASSERT_EQUAL_UINT16(1500u, input.get(1));
  TEST_ASSERT_EQUAL_UINT16(1425u, input.get(2));
  TEST_ASSERT_EQUAL_UINT16(1500u, input.get(3));
  TEST_ASSERT_EQUAL_UINT16(1000u, input.get(4));
  TEST_ASSERT_EQUAL_UINT16(1000u, input.get(5));
}

void test_input_crsf_rc_prefix()
{
  InputCRSF input;
  CrsfFrame frame;
  memset(&frame, 0, sizeof(frame));

  input.begin(nullptr);

  // prefix with few random bytes
  const uint8_t data[] = {
    0xA1, 0x04, 0xC5, 0x09,
    0xC8, 0x18, 0x16, 0xE0, 0x03, 0xDF, 0xD9, 0xC0, 0xF7, 0x8B, 0x5F, 0x94, 0xAF,
    0x7C, 0xE5, 0x2B, 0x5F, 0xF9, 0xCA, 0x07, 0x00, 0x00, 0x4C, 0x7C, 0xE2, 0x23
  };
  for (size_t i = 0; i < sizeof(data); i++) {
    input.parse(frame, data[i]);
  }

  const uint8_t crc = input.frameCrc(frame);
  TEST_ASSERT_EQUAL_UINT8(0x23, crc);

  TEST_ASSERT_EQUAL_UINT8(CRSF_ADDRESS_FLIGHT_CONTROLLER, frame.message.addr);
  TEST_ASSERT_EQUAL_UINT8(0x18, frame.message.size);
  TEST_ASSERT_EQUAL_UINT8(CRSF_FRAMETYPE_RC_CHANNELS_PACKED, frame.message.type);

  TEST_ASSERT_EQUAL_UINT16(1500, input.get(0));
  TEST_ASSERT_EQUAL_UINT16(1500, input.get(1));
}


int main(int argc, char **argv)
{
  UNITY_BEGIN();
  RUN_TEST(test_input_crsf_rc_valid);
  RUN_TEST(test_input_crsf_rc_prefix);
  UNITY_END();

  return 0;
}