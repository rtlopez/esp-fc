#include <unity.h>
#include <ArduinoFake.h>
#include "Device/InputCRSF.h"
#include "Device/InputIBUS.hpp"
#include "msp/msp_protocol.h"
#include <Gps.hpp>

using namespace Espfc;
using namespace Espfc::Device;
using namespace Espfc::Rc;
using namespace fakeit;

void test_input_crsf_rc_valid()
{
  InputCRSF input;
  CrsfMessage frame;
  memset(&frame, 0, sizeof(frame));
  uint8_t * frame_data = reinterpret_cast<uint8_t*>(&frame);

  When(Method(ArduinoFake(), micros)).Return(0);

  input.begin(nullptr, nullptr);

  const uint8_t data[] = {
    0xC8, 0x18, 0x16, 0xE0, 0x03, 0xDF, 0xD9, 0xC0, 0xF7, 0x8B, 0x5F, 0x94, 0xAF,
    0x7C, 0xE5, 0x2B, 0x5F, 0xF9, 0xCA, 0x07, 0x00, 0x00, 0x4C, 0x7C, 0xE2, 0x23
  };
  for (size_t i = 0; i < sizeof(data); i++) {
    input.parse(frame, data[i]);
  }

  for (size_t i = 0; i < sizeof(data); i++) {
    TEST_ASSERT_EQUAL_UINT8(data[i], frame_data[i]);
  }

  const uint8_t crc = Crsf::crc(frame);
  TEST_ASSERT_EQUAL_UINT8(0x23, crc);
  TEST_ASSERT_EQUAL_UINT8(0x23, frame.crc());

  TEST_ASSERT_EQUAL_UINT8(CRSF_ADDRESS_FLIGHT_CONTROLLER, frame.addr);
  TEST_ASSERT_EQUAL_UINT8(0x18, frame.size);
  TEST_ASSERT_EQUAL_UINT8(CRSF_FRAMETYPE_RC_CHANNELS_PACKED, frame.type);

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
  CrsfMessage frame;
  memset(&frame, 0, sizeof(frame));

  When(Method(ArduinoFake(), micros)).Return(0);

  input.begin(nullptr, nullptr);

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

  TEST_ASSERT_EQUAL_UINT8(CRSF_ADDRESS_FLIGHT_CONTROLLER, frame.addr);
  TEST_ASSERT_EQUAL_UINT8(0x18, frame.size);
  TEST_ASSERT_EQUAL_UINT8(CRSF_FRAMETYPE_RC_CHANNELS_PACKED, frame.type);

  TEST_ASSERT_EQUAL_UINT16(1500, input.get(0));
  TEST_ASSERT_EQUAL_UINT16(1500, input.get(1));
}

void test_crsf_encode_rc()
{
  CrsfMessage frame;
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

  uint8_t * frame_data = reinterpret_cast<uint8_t*>(&frame);

  TEST_ASSERT_EQUAL_UINT8(expected[0], frame_data[0]); // addr
  TEST_ASSERT_EQUAL_UINT8(expected[1], frame_data[1]); // size
  TEST_ASSERT_EQUAL_UINT8(expected[2], frame_data[2]); // type
  TEST_ASSERT_EQUAL_UINT8(expected[3], frame_data[3]);
  TEST_ASSERT_EQUAL_UINT8(expected[4], frame_data[4]);
  TEST_ASSERT_EQUAL_UINT8(expected[5], frame_data[5]);
  TEST_ASSERT_EQUAL_UINT8(expected[6], frame_data[6]);
  TEST_ASSERT_EQUAL_UINT8(expected[7], frame_data[7]);
  TEST_ASSERT_EQUAL_UINT8(expected[8], frame_data[8]);
  TEST_ASSERT_EQUAL_UINT8(expected[9], frame_data[9]);
  TEST_ASSERT_EQUAL_UINT8(expected[10], frame_data[10]);
  TEST_ASSERT_EQUAL_UINT8(expected[11], frame_data[11]);
  TEST_ASSERT_EQUAL_UINT8(expected[12], frame_data[12]);
  TEST_ASSERT_EQUAL_UINT8(expected[13], frame_data[13]);
  TEST_ASSERT_EQUAL_UINT8(expected[14], frame_data[14]);
  TEST_ASSERT_EQUAL_UINT8(expected[15], frame_data[15]);
  TEST_ASSERT_EQUAL_UINT8(expected[16], frame_data[16]);
  TEST_ASSERT_EQUAL_UINT8(expected[17], frame_data[17]);
  TEST_ASSERT_EQUAL_UINT8(expected[18], frame_data[18]);
  TEST_ASSERT_EQUAL_UINT8(expected[19], frame_data[19]);
  TEST_ASSERT_EQUAL_UINT8(expected[20], frame_data[20]);
  TEST_ASSERT_EQUAL_UINT8(expected[21], frame_data[21]);
  TEST_ASSERT_EQUAL_UINT8(expected[22], frame_data[22]);
  TEST_ASSERT_EQUAL_UINT8(expected[23], frame_data[23]);
  TEST_ASSERT_EQUAL_UINT8(expected[24], frame_data[24]);
  TEST_ASSERT_EQUAL_UINT8(expected[25], frame_data[25]); // crc

  const uint8_t crc = Crsf::crc(frame);
  TEST_ASSERT_EQUAL_UINT8(0xdb, crc);

  TEST_ASSERT_EQUAL_UINT8(CRSF_ADDRESS_FLIGHT_CONTROLLER, frame.addr);
  TEST_ASSERT_EQUAL_UINT8(0x18, frame.size);
  TEST_ASSERT_EQUAL_UINT8(CRSF_FRAMETYPE_RC_CHANNELS_PACKED, frame.type);
}

void test_crsf_decode_rc_struct()
{
  CrsfMessage frame;
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

  Crsf::decodeRcData(channels, (const CrsfData*)frame.payload);

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
  CrsfMessage frame;
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

  Crsf::decodeRcDataShift8(channels, (const CrsfData*)frame.payload);

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
  CrsfMessage frame;
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

void test_crsf_encode_msp_v1()
{
  CrsfMessage frame;
  memset(&frame, 0, sizeof(frame));

  Connect::MspResponse resp;
  resp.version = Connect::MSP_V1;
  resp.cmd = MSP_API_VERSION;
  resp.result = 0;
  resp.writeU8(1);
  resp.writeU8(2);
  resp.writeU8(3);

  Crsf::encodeMsp(frame, resp, CRSF_ADDRESS_RADIO_TRANSMITTER);

  // crsf headers
  TEST_ASSERT_EQUAL_UINT8(CRSF_ADDRESS_FLIGHT_CONTROLLER, frame.addr);
  TEST_ASSERT_EQUAL_UINT8(10, frame.size);
  TEST_ASSERT_EQUAL_UINT8(CRSF_FRAMETYPE_MSP_RESP, frame.type);

  // crsf ext headers
  TEST_ASSERT_EQUAL_UINT8(0xEA, frame.payload[0]); // radio-transmitter
  TEST_ASSERT_EQUAL_UINT8(0xC8, frame.payload[1]); // FC
  TEST_ASSERT_EQUAL_UINT8(0x30, frame.payload[2]); // status

  // ext msp v1 header
  TEST_ASSERT_EQUAL_UINT8(3, frame.payload[3]); // size
  TEST_ASSERT_EQUAL_UINT8(1, frame.payload[4]); // type // api_version(1)

  // ext msp payload
  TEST_ASSERT_EQUAL_UINT8(1, frame.payload[5]); // param1
  TEST_ASSERT_EQUAL_UINT8(2, frame.payload[6]); // param2
  TEST_ASSERT_EQUAL_UINT8(3, frame.payload[7]); // param3

  // crsf crc
  TEST_ASSERT_EQUAL_UINT8(0x6D, frame.crc());
}

void test_crsf_encode_msp_v2()
{
  CrsfMessage frame;
  memset(&frame, 0, sizeof(frame));

  Connect::MspResponse resp;
  resp.version = Connect::MSP_V2;
  resp.cmd = MSP_API_VERSION;
  resp.result = 0;
  resp.writeU8(1);
  resp.writeU8(2);
  resp.writeU8(3);

  Crsf::encodeMsp(frame, resp, CRSF_ADDRESS_RADIO_TRANSMITTER);

  // crsf headers
  TEST_ASSERT_EQUAL_UINT8(CRSF_ADDRESS_FLIGHT_CONTROLLER, frame.addr);
  TEST_ASSERT_EQUAL_UINT8(13, frame.size);
  TEST_ASSERT_EQUAL_UINT8(CRSF_FRAMETYPE_MSP_RESP, frame.type);

  // crsf ext headers
  TEST_ASSERT_EQUAL_UINT8(0xEA, frame.payload[0]); // radio-transmitter addr
  TEST_ASSERT_EQUAL_UINT8(0xC8, frame.payload[1]); // FC addr
  TEST_ASSERT_EQUAL_UINT8(0x50, frame.payload[2]); // status flags

  // ext msp v2 header
  TEST_ASSERT_EQUAL_UINT8(0, frame.payload[3]); // flags
  TEST_ASSERT_EQUAL_UINT8(1, frame.payload[4]); // type: api_version(1) (lo)
  TEST_ASSERT_EQUAL_UINT8(0, frame.payload[5]); // type: api_version(1) (hi)
  TEST_ASSERT_EQUAL_UINT8(3, frame.payload[6]); // size (lo)
  TEST_ASSERT_EQUAL_UINT8(0, frame.payload[7]); // size (hi)

  // ext msp payload
  TEST_ASSERT_EQUAL_UINT8(1, frame.payload[8]); // param1
  TEST_ASSERT_EQUAL_UINT8(2, frame.payload[9]); // param2
  TEST_ASSERT_EQUAL_UINT8(3, frame.payload[10]); // param3

  // crsf crc
  TEST_ASSERT_EQUAL_UINT8(0xF8, frame.crc());
}

void test_crsf_decode_msp_v1()
{
   const uint8_t data[] = {
    0xc8, 0x08, 0x7a, 0xc8, 0xea, 0x32, 0x00, 0x70, 0x70, 0x4b
  };
  CrsfMessage frame; 
  std::copy_n(data, sizeof(data), (uint8_t*)&frame);

  Connect::MspMessage m;
  uint8_t origin = 0;

  Crsf::decodeMsp(frame, m, origin);

  // crsf headers
  TEST_ASSERT_EQUAL_UINT8(CRSF_ADDRESS_FLIGHT_CONTROLLER, frame.addr);
  TEST_ASSERT_EQUAL_UINT8(0x08, frame.size);
  TEST_ASSERT_EQUAL_UINT8(CRSF_FRAMETYPE_MSP_REQ, frame.type);

  // crsf ext headers
  TEST_ASSERT_EQUAL_UINT8(0xC8, frame.payload[0]); // FC addr
  TEST_ASSERT_EQUAL_UINT8(0xEA, frame.payload[1]); // radio-transmitter addr (origin)
  TEST_ASSERT_EQUAL_UINT8(0x32, frame.payload[2]); // status flags

  // ext msp v2 header
  TEST_ASSERT_EQUAL_UINT8(0x00, frame.payload[3]); // size
  TEST_ASSERT_EQUAL_UINT8(0x70, frame.payload[4]); // type: msp_pid(0x70)

  // crsf crc
  TEST_ASSERT_EQUAL_UINT8(0x4B, frame.crc());

  // origin
  TEST_ASSERT_EQUAL_UINT8(0xEA, origin);
}

void test_input_ibus_rc_valid()
{
  InputIBUS input;
  InputIBUS::IBusData frame;
  memset(&frame, 0, sizeof(frame));
  uint8_t * frame_data = reinterpret_cast<uint8_t*>(&frame);

  When(Method(ArduinoFake(), micros)).Return(0);

  input.begin(nullptr);

  // const uint8_t data[] = {
  //   0x20, 0x40,  // preambule (len, cmd)
  //   0xDC, 0x05,  0xDC, 0x05,  0xBE, 0x05,  0xDC, 0x05, // channel 1-4
  //   0xD0, 0x07,  0xD0, 0x07,  0xDC, 0x05,  0xDC, 0x05, // channel 5-8
  //   0xDC, 0x05,  0xDC, 0x05,  0xDC, 0x05,  0xDC, 0x05, // channel 9-12
  //   0xDC, 0x05,  0xDC, 0x05,  // channel 13-14
  //   0x83, 0xF3  // checksum
  // };

  const uint8_t data[] = {
    0x20, 0x40,
    0xDB, 0x05, 0xDC, 0x05,  0x54, 0x05, 0xDC, 0x05,  0xE8, 0x03, 0xD0, 0x07,  0xD2, 0x05, 0xE8, 0x03,
    0xDC, 0x05, 0xDC, 0x05,  0xDC, 0x05, 0xDC, 0x05,  0xDC, 0x05, 0xDC, 0x05,
    0xDA, 0xF3,
  };
  for (size_t i = 0; i < sizeof(data); i++) {
    input.parse(frame, data[i]);
  }

  for (size_t i = 0; i < sizeof(data); i++) {
    TEST_ASSERT_EQUAL_HEX8(data[i], frame_data[i]);
  }

  TEST_ASSERT_EQUAL_HEX16(0xF3DA, frame.checksum);

  TEST_ASSERT_EQUAL_HEX8(0x20, frame.len);
  TEST_ASSERT_EQUAL_HEX8(0x40, frame.cmd);

  TEST_ASSERT_EQUAL_UINT16(1499, frame.ch[0]);
  TEST_ASSERT_EQUAL_UINT16(1500, frame.ch[1]);
  TEST_ASSERT_EQUAL_UINT16(1364, frame.ch[2]);
  TEST_ASSERT_EQUAL_UINT16(1500, frame.ch[3]);
  TEST_ASSERT_EQUAL_UINT16(1000, frame.ch[4]);
  TEST_ASSERT_EQUAL_UINT16(2000, frame.ch[5]);
  TEST_ASSERT_EQUAL_UINT16(1490, frame.ch[6]);
  TEST_ASSERT_EQUAL_UINT16(1000, frame.ch[7]);
  TEST_ASSERT_EQUAL_UINT16(1500, frame.ch[8]);
  TEST_ASSERT_EQUAL_UINT16(1500, frame.ch[9]);
  TEST_ASSERT_EQUAL_UINT16(1500, frame.ch[10]);
  TEST_ASSERT_EQUAL_UINT16(1500, frame.ch[11]);
  TEST_ASSERT_EQUAL_UINT16(1500, frame.ch[12]);
  TEST_ASSERT_EQUAL_UINT16(1500, frame.ch[13]);

  TEST_ASSERT_EQUAL_UINT16(1499, input.get(0));
  TEST_ASSERT_EQUAL_UINT16(1500, input.get(1));
  TEST_ASSERT_EQUAL_UINT16(1364, input.get(2));
  TEST_ASSERT_EQUAL_UINT16(1500, input.get(3));
  TEST_ASSERT_EQUAL_UINT16(1000, input.get(4));
  TEST_ASSERT_EQUAL_UINT16(2000, input.get(5));
  TEST_ASSERT_EQUAL_UINT16(1490, input.get(6));
  TEST_ASSERT_EQUAL_UINT16(1000, input.get(7));
  TEST_ASSERT_EQUAL_UINT16(1500, input.get(8));
  TEST_ASSERT_EQUAL_UINT16(1500, input.get(9));
  TEST_ASSERT_EQUAL_UINT16(1500, input.get(10));
  TEST_ASSERT_EQUAL_UINT16(1500, input.get(11));
  TEST_ASSERT_EQUAL_UINT16(1500, input.get(12));
  TEST_ASSERT_EQUAL_UINT16(1500, input.get(13));
}

int main(int argc, char **argv)
{
  UNITY_BEGIN();
  RUN_TEST(test_input_crsf_rc_valid);
  RUN_TEST(test_input_crsf_rc_prefix);
  RUN_TEST(test_crsf_encode_rc);
  RUN_TEST(test_crsf_decode_rc_struct);
  RUN_TEST(test_crsf_decode_rc_shift8);
  //RUN_TEST(test_crsf_decode_rc_shift32);
  RUN_TEST(test_crsf_encode_msp_v1);
  RUN_TEST(test_crsf_encode_msp_v2);
  RUN_TEST(test_crsf_decode_msp_v1);
  RUN_TEST(test_input_ibus_rc_valid);

  return UNITY_END();
}