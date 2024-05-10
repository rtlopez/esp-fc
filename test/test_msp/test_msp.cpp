#include <unity.h>
#include <ArduinoFake.h>
#include <Hal/Gpio.h>
#include <helper_3dmath.h>
#include <Kalman.h>
#include <EscDriver.h>
#include <printf.h>
#include "Msp/Msp.h"
#include "Msp/MspParser.h"

using namespace fakeit;
using namespace Espfc;

/*void setUp(void)
{
  ArduinoFakeReset();
}*/

// void tearDown(void) {
// // clean stuff up here
// }

using namespace Espfc::Msp;

#define MSP_V2_FLAG 0

void test_msp_v1_parse_header()
{
  MspMessage msg;
  MspParser parser;
  const uint8_t data[] = { '$', 'M' , '<' };
  for(size_t i = 0; i < sizeof(data); i++)
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
  const uint8_t data[] = { '$', 'M' , '<', 0, MSP_API_VERSION, 1 };
  for(size_t i = 0; i < sizeof(data); i++)
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
  const uint8_t data[] = { '$', 'M' , '<', 2, MSP_API_VERSION, 1, 2, 0 };
  for(size_t i = 0; i < sizeof(data); i++)
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
  const uint8_t data[] = { '$', 'X' , '<' };
  for(size_t i = 0; i < sizeof(data); i++)
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
  const uint8_t data[] = { '$', 'X' , '<', MSP_V2_FLAG, MSP_API_VERSION, 0, 0, 0, 69 };
  for(size_t i = 0; i < sizeof(data); i++)
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
  const uint8_t data[] = { '$', 'X' , '<', MSP_V2_FLAG, MSP_API_VERSION, 0, 2, 0, 1, 2, 102 };
  for(size_t i = 0; i < sizeof(data); i++)
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

int main(int argc, char **argv)
{
  UNITY_BEGIN();
  RUN_TEST(test_msp_v1_parse_header);
  RUN_TEST(test_msp_v1_parse_no_payload);
  RUN_TEST(test_msp_v1_parse_payload);
  RUN_TEST(test_msp_v2_parse_header);
  RUN_TEST(test_msp_v2_parse_no_payload);
  RUN_TEST(test_msp_v2_parse_payload);
  UNITY_END();

  return 0;
}