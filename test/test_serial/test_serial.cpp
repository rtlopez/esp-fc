#include "MockSerialStream.hpp"
#include "SerialManager.hpp"
#include "msp/msp_protocol.h"
#include <ArduinoFake.h>
#include <unity.h>

using namespace fakeit;
using namespace Espfc;

static const std::vector<uint8_t> mspApiVersionRequest = {'$', 'M', '<', 0, MSP_API_VERSION, MSP_API_VERSION};

void setUp()
{
  ArduinoFakeReset();
  When(Method(ArduinoFake(), millis)).AlwaysReturn(0);
  When(Method(ArduinoFake(), micros)).AlwaysReturn(0);
}

static void runCycle(SerialManager& manager)
{
  for (int i = 0; i < SERIAL_UART_COUNT; i++)
  {
    manager.update();
  }
}

void test_serial_begin_attaches_stream_to_configured_port()
{
  Model model;
  TelemetryManager telemetry(model);
  SerialManager manager(model, telemetry);
  model.config.serial[SERIAL_UART_0].functionMask = SERIAL_FUNCTION_MSP;

  manager.begin();

  TEST_ASSERT_NOT_NULL(model.state.serial[SERIAL_UART_0].stream);
}

void test_serial_begin_skips_port_without_function()
{
  Model model;
  TelemetryManager telemetry(model);
  SerialManager manager(model, telemetry);
  model.config.serial[SERIAL_UART_0].functionMask = 0;

  manager.begin();

  TEST_ASSERT_NULL(model.state.serial[SERIAL_UART_0].stream);
}

void test_serial_begin_skips_port_without_baud()
{
  Model model;
  TelemetryManager telemetry(model);
  SerialManager manager(model, telemetry);
  model.config.serial[SERIAL_UART_0].functionMask = SERIAL_FUNCTION_MSP;
  model.config.serial[SERIAL_UART_0].baud = SERIAL_SPEED_NONE;

  manager.begin();

  TEST_ASSERT_NULL(model.state.serial[SERIAL_UART_0].stream);
}

void test_serial_update_without_stream_is_safe()
{
  Model model;
  TelemetryManager telemetry(model);
  SerialManager manager(model, telemetry);

  TEST_ASSERT_EQUAL_INT(1, manager.update());
  TEST_ASSERT_EQUAL_INT(1, manager.update());
  TEST_ASSERT_EQUAL_INT(1, manager.update());
}

void test_serial_update_responds_to_msp_request()
{
  Model model;
  TelemetryManager telemetry(model);
  SerialManager manager(model, telemetry);
  MockSerialStream port;
  model.config.serial[SERIAL_UART_0].functionMask = SERIAL_FUNCTION_MSP;
  manager.begin();
  model.state.serial[SERIAL_UART_0].stream = &port;
  port.rx.assign(mspApiVersionRequest.begin(), mspApiVersionRequest.end());

  runCycle(manager);

  TEST_ASSERT_TRUE(port.data.size() > 3);
  TEST_ASSERT_EQUAL_UINT8('$', port.data[0]);
  TEST_ASSERT_EQUAL_UINT8('M', port.data[1]);
  TEST_ASSERT_EQUAL_UINT8('>', port.data[2]);
  TEST_ASSERT_EQUAL_UINT8(MSP_API_VERSION, port.data[4]);
  TEST_ASSERT_EQUAL_size_t(0, port.rx.size());
}

void test_serial_update_ignores_msp_when_function_not_set()
{
  Model model;
  TelemetryManager telemetry(model);
  SerialManager manager(model, telemetry);
  MockSerialStream port;
  model.config.serial[SERIAL_UART_0].functionMask = SERIAL_FUNCTION_BLACKBOX;
  manager.begin();
  model.state.serial[SERIAL_UART_0].stream = &port;
  port.rx.assign(mspApiVersionRequest.begin(), mspApiVersionRequest.end());

  runCycle(manager);

  TEST_ASSERT_EQUAL_size_t(0, port.data.size());
  TEST_ASSERT_EQUAL_size_t(mspApiVersionRequest.size(), port.rx.size());
}

void test_serial_update_skips_port_used_as_serial_rx()
{
  Model model;
  TelemetryManager telemetry(model);
  SerialManager manager(model, telemetry);
  MockSerialStream port;
  model.config.serial[SERIAL_UART_0].functionMask = SERIAL_FUNCTION_RX_SERIAL | SERIAL_FUNCTION_MSP;
  manager.begin();
  model.state.serial[SERIAL_UART_0].stream = &port;
  port.rx.assign(mspApiVersionRequest.begin(), mspApiVersionRequest.end());

  runCycle(manager);

  TEST_ASSERT_EQUAL_size_t(0, port.data.size());
  TEST_ASSERT_EQUAL_size_t(mspApiVersionRequest.size(), port.rx.size());
}

int main(int argc, char** argv)
{
  UNITY_BEGIN();
  RUN_TEST(test_serial_begin_attaches_stream_to_configured_port);
  RUN_TEST(test_serial_begin_skips_port_without_function);
  RUN_TEST(test_serial_begin_skips_port_without_baud);
  RUN_TEST(test_serial_update_without_stream_is_safe);
  RUN_TEST(test_serial_update_responds_to_msp_request);
  RUN_TEST(test_serial_update_ignores_msp_when_function_not_set);
  RUN_TEST(test_serial_update_skips_port_used_as_serial_rx);
  return UNITY_END();
}
