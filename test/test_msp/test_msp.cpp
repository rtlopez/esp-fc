#include "Connect/Msp.hpp"
#include "Connect/MspParser.hpp"
#include "msp/msp_protocol.h"
#include <EscDriver.hpp>
#include <Gps.hpp>
#include <Hal/Gpio.hpp>
#include <helper_3dmath.hpp>
#include <printf.h>
#include <unity.h>

using namespace Espfc;
using namespace Espfc::Connect;

/*void setUp(void)
{
  ArduinoFakeReset();
}*/

// void tearDown(void) {
// // clean stuff up here
// }

#define MSP_V2_FLAG 0

void test_msp_v1_parse_header()
{
  MspMessage msg;
  MspParser parser;
  const uint8_t data[] = {'$', 'M', '<'};
  for (size_t i = 0; i < sizeof(data); i++)
  {
    parser.parse(data[i], msg);
  }
  TEST_ASSERT_EQUAL(MSP_TYPE_CMD, msg.dir);
  TEST_ASSERT_EQUAL(MSP_V1, msg.version);
}

void test_msp_v1_parse_no_payload()
{
  MspMessage msg;
  MspParser parser;
  const uint8_t data[] = {'$', 'M', '<', 0, MSP_API_VERSION, 1};
  for (size_t i = 0; i < sizeof(data); i++)
  {
    parser.parse(data[i], msg);
  }
  TEST_ASSERT_EQUAL_INT(MSP_TYPE_CMD, msg.dir);
  TEST_ASSERT_EQUAL_INT(MSP_API_VERSION, msg.cmd);
  TEST_ASSERT_EQUAL_UINT16(0, msg.received);
  TEST_ASSERT_EQUAL_INT(0, msg.remain());
  TEST_ASSERT_EQUAL_UINT8(1, msg.checksum);
  TEST_ASSERT_EQUAL_UINT8(MSP_STATE_RECEIVED, msg.state);
}

void test_msp_v1_parse_payload()
{
  MspMessage msg;
  MspParser parser;
  const uint8_t data[] = {'$', 'M', '<', 2, MSP_API_VERSION, 1, 2, 0};
  for (size_t i = 0; i < sizeof(data); i++)
  {
    parser.parse(data[i], msg);
  }
  TEST_ASSERT_EQUAL_INT(MSP_TYPE_CMD, msg.dir);
  TEST_ASSERT_EQUAL_INT(MSP_API_VERSION, msg.cmd);
  TEST_ASSERT_EQUAL_UINT16(2, msg.received);
  TEST_ASSERT_EQUAL_INT(2, msg.remain());
  TEST_ASSERT_EQUAL_UINT8(0, msg.checksum);
  TEST_ASSERT_EQUAL_UINT8(MSP_STATE_RECEIVED, msg.state);
}

void test_msp_v2_parse_header()
{
  MspMessage msg;
  MspParser parser;
  const uint8_t data[] = {'$', 'X', '<'};
  for (size_t i = 0; i < sizeof(data); i++)
  {
    parser.parse(data[i], msg);
  }
  TEST_ASSERT_EQUAL(MSP_TYPE_CMD, msg.dir);
  TEST_ASSERT_EQUAL(MSP_V2, msg.version);
}

void test_msp_v2_parse_no_payload()
{
  MspMessage msg;
  MspParser parser;
  const uint8_t data[] = {'$', 'X', '<', MSP_V2_FLAG, MSP_API_VERSION, 0, 0, 0, 69};
  for (size_t i = 0; i < sizeof(data); i++)
  {
    parser.parse(data[i], msg);
  }
  TEST_ASSERT_EQUAL_INT(MSP_TYPE_CMD, msg.dir);
  TEST_ASSERT_EQUAL_INT(MSP_API_VERSION, msg.cmd);
  TEST_ASSERT_EQUAL_UINT16(0, msg.received);
  TEST_ASSERT_EQUAL_INT(0, msg.remain());
  TEST_ASSERT_EQUAL_UINT8(69, msg.checksum2);
  TEST_ASSERT_EQUAL_UINT8(MSP_STATE_RECEIVED, msg.state);
}

void test_msp_v2_parse_payload()
{
  MspMessage msg;
  MspParser parser;
  const uint8_t data[] = {'$', 'X', '<', MSP_V2_FLAG, MSP_API_VERSION, 0, 2, 0, 1, 2, 102};
  for (size_t i = 0; i < sizeof(data); i++)
  {
    parser.parse(data[i], msg);
  }
  TEST_ASSERT_EQUAL_INT(MSP_TYPE_CMD, msg.dir);
  TEST_ASSERT_EQUAL_INT(MSP_API_VERSION, msg.cmd);
  TEST_ASSERT_EQUAL_UINT16(2, msg.received);
  TEST_ASSERT_EQUAL_INT(2, msg.remain());
  TEST_ASSERT_EQUAL_UINT8(102, msg.checksum2);
  TEST_ASSERT_EQUAL_UINT8(MSP_STATE_RECEIVED, msg.state);
}

void test_msp_message_append_bounds()
{
  MspMessage msg;
  uint8_t data[MSP_BUF_SIZE + 64];
  for (size_t i = 0; i < sizeof(data); i++)
  {
    data[i] = i;
  }
  msg.append(data, sizeof(data));
  TEST_ASSERT_EQUAL_UINT16(MSP_BUF_SIZE, msg.received);

  msg.append(data, 10);
  TEST_ASSERT_EQUAL_UINT16(MSP_BUF_SIZE, msg.received);
}

void test_msp_message_read_bounds()
{
  MspMessage msg;
  const uint8_t data[] = {1, 2};
  msg.append(data, sizeof(data));

  TEST_ASSERT_EQUAL_UINT8(1, msg.readU8());
  TEST_ASSERT_EQUAL_UINT8(2, msg.readU8());
  TEST_ASSERT_EQUAL_UINT8(0, msg.readU8());
  TEST_ASSERT_EQUAL_UINT32(0, msg.readU32());
  TEST_ASSERT_EQUAL_INT(0, msg.remain());

  msg.advance(255);
  TEST_ASSERT_EQUAL_INT(0, msg.remain());
  TEST_ASSERT_EQUAL_UINT16(2, msg.read);
}

void test_msp_response_write_bounds()
{
  MspResponse resp;
  for (size_t i = 0; i < MSP_BUF_OUT_SIZE + 64; i++)
  {
    resp.writeU8(i);
  }
  TEST_ASSERT_EQUAL_UINT16(MSP_BUF_OUT_SIZE, resp.len);
  TEST_ASSERT_EQUAL_INT(0, resp.remain());

  resp.advance(100);
  TEST_ASSERT_EQUAL_UINT16(MSP_BUF_OUT_SIZE, resp.len);
}

int main(int argc, char** argv)
{
  UNITY_BEGIN();
  RUN_TEST(test_msp_message_append_bounds);
  RUN_TEST(test_msp_message_read_bounds);
  RUN_TEST(test_msp_response_write_bounds);
  RUN_TEST(test_msp_v1_parse_header);
  RUN_TEST(test_msp_v1_parse_no_payload);
  RUN_TEST(test_msp_v1_parse_payload);
  RUN_TEST(test_msp_v2_parse_header);
  RUN_TEST(test_msp_v2_parse_no_payload);
  RUN_TEST(test_msp_v2_parse_payload);

  return UNITY_END();
}