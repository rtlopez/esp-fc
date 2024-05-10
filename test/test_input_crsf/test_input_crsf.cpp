#include <unity.h>
#include <ArduinoFake.h>
#include "Device/InputCRSF.h"
#include <EscDriver.h>
#include <helper_3dmath.h>
#include <Kalman.h>
#include "msp/msp_protocol.h"
#include <printf.h>

using namespace Espfc;
using namespace Espfc::Device;
using namespace Espfc::Rc;

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

  const uint8_t crc = Crsf::crc(frame);
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

  const uint8_t crc = Crsf::crc(frame);
  TEST_ASSERT_EQUAL_UINT8(0x23, crc);

  TEST_ASSERT_EQUAL_UINT8(CRSF_ADDRESS_FLIGHT_CONTROLLER, frame.message.addr);
  TEST_ASSERT_EQUAL_UINT8(0x18, frame.message.size);
  TEST_ASSERT_EQUAL_UINT8(CRSF_FRAMETYPE_RC_CHANNELS_PACKED, frame.message.type);

  TEST_ASSERT_EQUAL_UINT16(1500, input.get(0));
  TEST_ASSERT_EQUAL_UINT16(1500, input.get(1));
}

void test_crsf_encode_rc()
{
  CrsfFrame frame;
  memset(&frame, 0, sizeof(frame));

  CrsfData data;
  data.chan0 = 992;
  data.chan1 = 992;
  data.chan2 = 172;
  data.chan3 = 992;
  data.chan4 = 992;
  data.chan5 = 992;
  data.chan6 = 992;
  data.chan7 = 992;
  data.chan8 = 992;
  data.chan9 = 992;
  data.chan10 = 992;
  data.chan11 = 992;
  data.chan12 = 992;
  data.chan13 = 992;
  data.chan14 = 992;
  data.chan15 = 992;

  Crsf::encodeRcData(frame, data);

  const uint8_t expected[] = {
    0xC8, 0x18, 0x16, 0xE0, 0x03, 0x1F, 0x2B, 0xC0, 0x07, 0x3E, 0xF0, 0x81, 0x0F,
    0x7C, 0xE0, 0x03, 0x1F, 0xF8, 0xC0, 0x07, 0x3E, 0xF0, 0x81, 0x0F, 0x7C, 0xDB
  };

  TEST_ASSERT_EQUAL_UINT8(expected[0], frame.data[0]); // addr
  TEST_ASSERT_EQUAL_UINT8(expected[1], frame.data[1]); // size
  TEST_ASSERT_EQUAL_UINT8(expected[2], frame.data[2]); // type
  TEST_ASSERT_EQUAL_UINT8(expected[3], frame.data[3]);
  TEST_ASSERT_EQUAL_UINT8(expected[4], frame.data[4]);
  TEST_ASSERT_EQUAL_UINT8(expected[5], frame.data[5]);
  TEST_ASSERT_EQUAL_UINT8(expected[6], frame.data[6]);
  TEST_ASSERT_EQUAL_UINT8(expected[7], frame.data[7]);
  TEST_ASSERT_EQUAL_UINT8(expected[8], frame.data[8]);
  TEST_ASSERT_EQUAL_UINT8(expected[9], frame.data[9]);
  TEST_ASSERT_EQUAL_UINT8(expected[10], frame.data[10]);
  TEST_ASSERT_EQUAL_UINT8(expected[11], frame.data[11]);
  TEST_ASSERT_EQUAL_UINT8(expected[12], frame.data[12]);
  TEST_ASSERT_EQUAL_UINT8(expected[13], frame.data[13]);
  TEST_ASSERT_EQUAL_UINT8(expected[14], frame.data[14]);
  TEST_ASSERT_EQUAL_UINT8(expected[15], frame.data[15]);
  TEST_ASSERT_EQUAL_UINT8(expected[16], frame.data[16]);
  TEST_ASSERT_EQUAL_UINT8(expected[17], frame.data[17]);
  TEST_ASSERT_EQUAL_UINT8(expected[18], frame.data[18]);
  TEST_ASSERT_EQUAL_UINT8(expected[19], frame.data[19]);
  TEST_ASSERT_EQUAL_UINT8(expected[20], frame.data[20]);
  TEST_ASSERT_EQUAL_UINT8(expected[21], frame.data[21]);
  TEST_ASSERT_EQUAL_UINT8(expected[22], frame.data[22]);
  TEST_ASSERT_EQUAL_UINT8(expected[23], frame.data[23]);
  TEST_ASSERT_EQUAL_UINT8(expected[24], frame.data[24]);
  TEST_ASSERT_EQUAL_UINT8(expected[25], frame.data[25]); // crc

  const uint8_t crc = Crsf::crc(frame);
  TEST_ASSERT_EQUAL_UINT8(0xdb, crc);

  TEST_ASSERT_EQUAL_UINT8(CRSF_ADDRESS_FLIGHT_CONTROLLER, frame.message.addr);
  TEST_ASSERT_EQUAL_UINT8(0x18, frame.message.size);
  TEST_ASSERT_EQUAL_UINT8(CRSF_FRAMETYPE_RC_CHANNELS_PACKED, frame.message.type);
}

void test_crsf_decode_rc_struct()
{
  CrsfFrame frame;
  memset(&frame, 0, sizeof(frame));

  CrsfData data;
  data.chan0 = 992;
  data.chan1 = 992;
  data.chan2 = 172;
  data.chan3 = 992;
  data.chan4 = 992;
  data.chan5 = 992;
  data.chan6 = 992;
  data.chan7 = 992;
  data.chan8 = 992;
  data.chan9 = 992;
  data.chan10 = 992;
  data.chan11 = 992;
  data.chan12 = 992;
  data.chan13 = 992;
  data.chan14 = 992;
  data.chan15 = 992;

  Crsf::encodeRcData(frame, data);

  uint16_t channels[16] = { 0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0};

  Crsf::decodeRcData(channels, (const CrsfData*)frame.message.payload);

  TEST_ASSERT_EQUAL_UINT16(1500, channels[0]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[1]);
  TEST_ASSERT_EQUAL_UINT16( 988, channels[2]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[3]);

  TEST_ASSERT_EQUAL_UINT16(1500, channels[4]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[5]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[6]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[7]);

  TEST_ASSERT_EQUAL_UINT16(1500, channels[8]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[9]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[10]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[11]);

  TEST_ASSERT_EQUAL_UINT16(1500, channels[12]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[13]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[14]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[15]);
}

void test_crsf_decode_rc_shift8()
{
  CrsfFrame frame;
  memset(&frame, 0, sizeof(frame));

  CrsfData data;
  data.chan0 = 992;
  data.chan1 = 992;
  data.chan2 = 172;
  data.chan3 = 992;
  data.chan4 = 992;
  data.chan5 = 992;
  data.chan6 = 992;
  data.chan7 = 992;
  data.chan8 = 992;
  data.chan9 = 992;
  data.chan10 = 992;
  data.chan11 = 992;
  data.chan12 = 992;
  data.chan13 = 992;
  data.chan14 = 992;
  data.chan15 = 992;

  Crsf::encodeRcData(frame, data);

  uint16_t channels[16] = { 0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0};

  Crsf::decodeRcDataShift8(channels, (const CrsfData*)frame.message.payload);

  TEST_ASSERT_EQUAL_UINT16(1500, channels[0]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[1]);
  TEST_ASSERT_EQUAL_UINT16( 988, channels[2]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[3]);

  TEST_ASSERT_EQUAL_UINT16(1500, channels[4]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[5]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[6]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[7]);

  TEST_ASSERT_EQUAL_UINT16(1500, channels[8]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[9]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[10]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[11]);

  TEST_ASSERT_EQUAL_UINT16(1500, channels[12]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[13]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[14]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[15]);
}

/*void test_crsf_decode_rc_shift32()
{
  CrsfFrame frame;
  memset(&frame, 0, sizeof(frame));

  CrsfData data;
  data.chan0 = 992;
  data.chan1 = 992;
  data.chan2 = 191;
  data.chan3 = 992;
  data.chan4 = 992;
  data.chan5 = 992;
  data.chan6 = 992;
  data.chan7 = 992;
  data.chan8 = 992;
  data.chan9 = 992;
  data.chan10 = 992;
  data.chan11 = 992;
  data.chan12 = 992;
  data.chan13 = 992;
  data.chan14 = 992;
  data.chan15 = 992;

  Crsf::encodeRcData(frame, data);

  uint16_t channels[16] = { 0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0};

  Crsf::decodeRcDataShift32(channels, (const CrsfData*)frame.message.payload);

  TEST_ASSERT_EQUAL_UINT16(1500, channels[0]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[1]);
  TEST_ASSERT_EQUAL_UINT16(1000, channels[2]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[3]);

  TEST_ASSERT_EQUAL_UINT16(1500, channels[4]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[5]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[6]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[7]);

  TEST_ASSERT_EQUAL_UINT16(1500, channels[8]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[9]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[10]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[11]);

  TEST_ASSERT_EQUAL_UINT16(1500, channels[12]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[13]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[14]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[15]);
}*/

int main(int argc, char **argv)
{
  UNITY_BEGIN();
  RUN_TEST(test_input_crsf_rc_valid);
  RUN_TEST(test_input_crsf_rc_prefix);
  RUN_TEST(test_crsf_encode_rc);
  RUN_TEST(test_crsf_decode_rc_struct);
  RUN_TEST(test_crsf_decode_rc_shift8);
  //RUN_TEST(test_crsf_decode_rc_shift32);
  UNITY_END();

  return 0;
}