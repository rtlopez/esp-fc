#include "Sensor/VoltageSensor.hpp"
#include <ArduinoFake.h>
#include <platform.h>
#include <unity.h>

using namespace Espfc;
using namespace Espfc::Sensor;

void setUpBattery(Model& model)
{
  model.state.battery.rawVoltage = 2000;
  model.state.battery.voltageUnfiltered = 12.4f;
  model.state.battery.voltage = 12.4f;
  model.state.battery.cellVoltage = 4.1f;
  model.state.battery.percentage = 90.0f;
  model.state.battery.cells = 3;
  model.state.battery.rawCurrent = 1000;
  model.state.battery.currentUnfiltered = 5.5f;
  model.state.battery.current = 5.5f;
}

void test_voltage_sensor_adc_change_reset_disabled()
{
  Model model;
  VoltageSensor sensor(model);
  setUpBattery(model);
  model.config.vbat.source = VBAT_SOURCE_NONE;
  model.config.ibat.source = IBAT_SOURCE_NONE;

  sensor.reload(MODEL_CHANGE_ADC);

  TEST_ASSERT_EQUAL_INT16(0, model.state.battery.rawVoltage);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, model.state.battery.voltageUnfiltered);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, model.state.battery.voltage);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, model.state.battery.cellVoltage);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, model.state.battery.percentage);
  TEST_ASSERT_EQUAL_INT8(0, model.state.battery.cells);
  TEST_ASSERT_EQUAL_INT16(0, model.state.battery.rawCurrent);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, model.state.battery.currentUnfiltered);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, model.state.battery.current);
}

void test_voltage_sensor_adc_change_keep_enabled()
{
  Model model;
  VoltageSensor sensor(model);
  setUpBattery(model);
  model.config.vbat.source = VBAT_SOURCE_ADC;
  model.config.ibat.source = IBAT_SOURCE_ADC;

  sensor.reload(MODEL_CHANGE_ADC);

  TEST_ASSERT_EQUAL_INT16(2000, model.state.battery.rawVoltage);
  TEST_ASSERT_EQUAL_FLOAT(12.4f, model.state.battery.voltage);
  TEST_ASSERT_EQUAL_INT8(3, model.state.battery.cells);
  TEST_ASSERT_EQUAL_INT16(1000, model.state.battery.rawCurrent);
  TEST_ASSERT_EQUAL_FLOAT(5.5f, model.state.battery.current);
  TEST_ASSERT_EQUAL_INT8(50, model.state.battery.samples); // cell count redetection
}

void test_voltage_sensor_adc_change_mixed()
{
  Model model;
  VoltageSensor sensor(model);
  setUpBattery(model);
  model.config.vbat.source = VBAT_SOURCE_ADC;
  model.config.ibat.source = IBAT_SOURCE_NONE;

  sensor.reload(MODEL_CHANGE_ADC);

  TEST_ASSERT_EQUAL_FLOAT(12.4f, model.state.battery.voltage);
  TEST_ASSERT_EQUAL_INT16(0, model.state.battery.rawCurrent);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, model.state.battery.current);
}

int main(int argc, char** argv)
{
  UNITY_BEGIN();
  RUN_TEST(test_voltage_sensor_adc_change_reset_disabled);
  RUN_TEST(test_voltage_sensor_adc_change_keep_enabled);
  RUN_TEST(test_voltage_sensor_adc_change_mixed);

  return UNITY_END();
}
