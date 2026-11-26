#include "Connect/Msp.hpp"
#include "Connect/MspParser.hpp"
#include "Connect/MspProcessor.hpp"
#include "MockSerialStream.hpp"
#include "msp/msp_protocol.h"
#include "msp/msp_protocol_v2_common.h"
#include <Gps.hpp>
#include <Hal/Gpio.hpp>
#include <helper_3dmath.hpp>
#include <printf.h>
#include <unity.h>
#include <vector>

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

static int processMsp(Model& model, uint16_t cmd, const std::vector<uint8_t>& payload)
{
  MspProcessor msp(model);
  MspMessage m;
  m.cmd = cmd;
  m.append(payload.data(), payload.size());
  MspResponse r;
  MockSerialStream serial;
  msp.processCommand(m, r, serial);
  return r.result;
}

void test_msp_set_cf_serial_config_known_port()
{
  Model model;
  const uint8_t baud = SERIAL_SPEED_INDEX_115200;
  processMsp(model, MSP_SET_CF_SERIAL_CONFIG, {SERIAL_ID_UART_1, SERIAL_FUNCTION_GPS, 0, baud, 0, 0, baud});

  TEST_ASSERT_EQUAL_UINT32(SERIAL_FUNCTION_GPS, model.config.serial[SERIAL_UART_0].functionMask);
  TEST_ASSERT_EQUAL_INT(SERIAL_SPEED_115200, model.config.serial[SERIAL_UART_0].baud);
  TEST_ASSERT_EQUAL_INT(SERIAL_SPEED_115200, model.config.serial[SERIAL_UART_0].blackboxBaud);
}

void test_msp_set_cf_serial_config_unknown_port_is_skipped()
{
  Model model;
  const auto before = model.config.serial[SERIAL_UART_0];
  const uint8_t baud = SERIAL_SPEED_INDEX_115200;
  const std::vector<uint8_t> unknown = {SERIAL_ID_UART_2, SERIAL_FUNCTION_GPS, 0, baud, 0, 0, baud};
  const std::vector<uint8_t> known = {SERIAL_ID_UART_1, SERIAL_FUNCTION_GPS, 0, baud, 0, 0, baud};
  std::vector<uint8_t> payload = unknown;
  payload.insert(payload.end(), known.begin(), known.end());
  processMsp(model, MSP_SET_CF_SERIAL_CONFIG, payload);

  TEST_ASSERT_EQUAL_UINT32(SERIAL_FUNCTION_GPS, model.config.serial[SERIAL_UART_0].functionMask);
  TEST_ASSERT_NOT_EQUAL_UINT32(before.functionMask, model.config.serial[SERIAL_UART_0].functionMask);
}

void test_msp_set_cf_serial_config_invalid_size()
{
  Model model;
  const auto before = model.config.serial[SERIAL_UART_0];
  const int result = processMsp(model, MSP_SET_CF_SERIAL_CONFIG, {SERIAL_ID_UART_1, SERIAL_FUNCTION_GPS, 0});

  TEST_ASSERT_EQUAL_INT(-1, result);
  TEST_ASSERT_EQUAL_UINT32(before.functionMask, model.config.serial[SERIAL_UART_0].functionMask);
}

void test_msp2_set_serial_config_known_and_unknown_port()
{
  Model model;
  const uint8_t baud = SERIAL_SPEED_INDEX_115200;
  processMsp(model, MSP2_COMMON_SET_SERIAL_CONFIG,
             {2, SERIAL_ID_UART_2, SERIAL_FUNCTION_GPS, 0, 0, 0, baud, 0, 0, baud, SERIAL_ID_UART_1,
              SERIAL_FUNCTION_GPS, 0, 0, 0, baud, 0, 0, baud});

  TEST_ASSERT_EQUAL_UINT32(SERIAL_FUNCTION_GPS, model.config.serial[SERIAL_UART_0].functionMask);
  TEST_ASSERT_EQUAL_INT(SERIAL_SPEED_115200, model.config.serial[SERIAL_UART_0].baud);
}

void test_msp2_set_serial_config_invalid_size()
{
  Model model;
  const int result = processMsp(model, MSP2_COMMON_SET_SERIAL_CONFIG, {1, SERIAL_ID_UART_1, 0});
  TEST_ASSERT_EQUAL_INT(-1, result);
}

int main(int argc, char** argv)
{
  UNITY_BEGIN();
  RUN_TEST(test_msp_set_cf_serial_config_known_port);
  RUN_TEST(test_msp_set_cf_serial_config_unknown_port_is_skipped);
  RUN_TEST(test_msp_set_cf_serial_config_invalid_size);
  RUN_TEST(test_msp2_set_serial_config_known_and_unknown_port);
  RUN_TEST(test_msp2_set_serial_config_invalid_size);
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