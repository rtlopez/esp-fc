#include "BaroBMP280.hpp"

#define BMP280_ADDRESS_FIRST 0x76
#define BMP280_ADDRESS_SECOND 0x77
#define BMP280_WHOAMI_ID 0x58

#define BMP280_WHOAMI_REG 0xD0
#define BMP280_VERSION_REG 0xD1
#define BMP280_RESET_REG 0xE0
#define BMP280_RESET_VAL 0xB6

#define BMP280_CALIB_REG 0x88
#define BMP280_CAL26_REG 0xE1 // R calibration stored in 0xE1-0xF0

#define BMP280_STATUS_REG 0xF3
#define BMP280_CONTROL_REG 0xF4
#define BMP280_CONFIG_REG 0xF5
#define BMP280_PRESSURE_REG 0xF7
#define BMP280_TEMPERATURE_REG 0xFA

#define BMP280_MODE_NORMAL 0x03

#define BMP280_FILTER_OFF 0x00
#define BMP280_FILTER_X2 0x01
#define BMP280_FILTER_X4 0x02
#define BMP280_FILTER_X8 0x03
#define BMP280_FILTER_X16 0x04

#define BMP280_SAMPLING_X1 1
#define BMP280_SAMPLING_X2 2
#define BMP280_SAMPLING_X4 3
#define BMP280_SAMPLING_X8 4

namespace Espfc::Device::Baro {

int BaroBMP280::begin(BusDevice* bus)
{
  return begin(bus, BMP280_ADDRESS_FIRST) ? 1 : begin(bus, BMP280_ADDRESS_SECOND) ? 1 : 0;
}

int BaroBMP280::begin(BusDevice* bus, uint8_t addr)
{
  setBus(bus, addr);

  if (!testConnection()) return 0;

  _bus->read(_addr, BMP280_CALIB_REG, sizeof(CalibrationData), (uint8_t*)&_cal); // read callibration

  writeReg(BMP280_RESET_REG, BMP280_RESET_VAL); // device reset
  delay(2);

  writeReg(BMP280_CONFIG_REG, BMP280_FILTER_X4 << 2); // set minimal standby and IIR filter X2
  // writeReg(BMP280_CONFIG_REG, 0); // set minimal standby and IIR filter off

  writeReg(BMP280_CONTROL_REG,
           BMP280_SAMPLING_X2 << 5 | BMP280_SAMPLING_X8 << 2 | BMP280_MODE_NORMAL); // set sampling mode

  delay(20);

  return 1;
}

BaroDeviceType BaroBMP280::getType() const
{
  return BARO_BMP280;
}

float BaroBMP280::readTemperature()
{
  float T = (_t_fine * 5 + 128) >> 8;
  return T * 0.01f;
}

float BaroBMP280::readPressure()
{
  readMesurment();

  int32_t adc_T = _raw_temp;
  adc_T >>= 4;

  int32_t vart1 = ((((adc_T >> 3) - ((int32_t)_cal.dig_T1 << 1))) * ((int32_t)_cal.dig_T2)) >> 11;
  int32_t vart2 = (((((adc_T >> 4) - ((int32_t)_cal.dig_T1)) * ((adc_T >> 4) - ((int32_t)_cal.dig_T1))) >> 12) *
                   ((int32_t)_cal.dig_T3)) >>
                  14;
  _t_fine = vart1 + vart2;
  //_t_fine += ((var1 + var2) - _t_fine + 4) >> 3; // smooth t_fine

  int32_t adc_P = _raw_pressure;
  adc_P >>= 4;

  int64_t var1 = ((int64_t)_t_fine) - 128000;
  int64_t var2 = var1 * var1 * (int64_t)_cal.dig_P6;
  var2 += ((var1 * (int64_t)_cal.dig_P5) << 17);
  var2 += (((int64_t)_cal.dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)_cal.dig_P3) >> 8) + ((var1 * (int64_t)_cal.dig_P2) << 12);
  var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)_cal.dig_P1) >> 33;

  if (var1 == 0)
  {
    return 0; // avoid exception caused by division by zero
  }
  int64_t p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)_cal.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)_cal.dig_P8) * p) >> 19;

  p = ((p + var1 + var2) >> 8) + (((int64_t)_cal.dig_P7) << 4);

  return ((float)p) / 256.f; // Pa
}

void BaroBMP280::setMode(BaroDeviceMode mode)
{
  (void)mode;
}

int BaroBMP280::getDelay(BaroDeviceMode mode) const
{
  switch (mode)
  {
    case BARO_MODE_TEMP: return 0;
    default:
      // return 5500; // if sapling X1
      // return 7500; // if sampling X2
      // return 11500;  // if sampling X4
      return 19500; // if sampling X8
                    // return 37500;  // if sampling X16
  }
}

bool BaroBMP280::testConnection()
{
  uint8_t whoami = 0;
  _bus->read(_addr, BMP280_WHOAMI_REG, 1, &whoami);
  return whoami == BMP280_WHOAMI_ID;
}

void BaroBMP280::readMesurment()
{
  uint8_t buffer[6] = {0, 0, 0, 0, 0, 0};
  _bus->readFast(_addr, BMP280_PRESSURE_REG, 6, buffer);
  _raw_pressure = buffer[2] | (buffer[1] << 8) | (buffer[0] << 16);
  _raw_temp = buffer[5] | (buffer[4] << 8) | (buffer[3] << 16);
}

int8_t BaroBMP280::writeReg(uint8_t reg, uint8_t val)
{
  return _bus->write(_addr, reg, 1, &val);
}

} // namespace Espfc::Device::Baro
