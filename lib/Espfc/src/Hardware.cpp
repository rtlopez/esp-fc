#include "Hardware.h"
#include "Device/GyroDevice.h"
#include "Device/GyroMPU6050.h"
#include "Device/GyroMPU6500.h"
#include "Device/GyroMPU9250.h"
#include "Device/GyroLSM6DSO.h"
#include "Device/GyroICM20602.h"
#include "Device/GyroBMI160.h"
#include "Device/MagHMC5338L.h"
#include "Device/MagQMC5338L.h"
#include "Device/MagQMC5338P.h"
#include "Device/MagAK8963.h"
#include "Device/BaroDevice.h"
#include "Device/BaroBMP085.h"
#include "Device/BaroBMP280.h"
#include "Device/BaroSPL06.h"
#if defined(ESPFC_WIFI_ALT)
#include <ESP8266WiFi.h>
#elif defined(ESPFC_WIFI)
#include <WiFi.h>
#endif

namespace {
#if defined(ESPFC_SPI_0)
  #if defined(ESP32C3) || defined(ESP32S3) || defined(ESP32S2)
    static SPIClass SPI1(HSPI);
  #elif defined(ESP32)
    static SPIClass SPI1(VSPI);
  #endif
  static Espfc::Device::BusSPI spiBus(ESPFC_SPI_0_DEV);
#endif
#if defined(ESPFC_I2C_0)
  static Espfc::Device::BusI2C i2cBus(WireInstance);
#endif
  static Espfc::Device::BusSlave gyroSlaveBus;
  static Espfc::Device::GyroMPU6050 mpu6050;
  static Espfc::Device::GyroMPU6500 mpu6500;
  static Espfc::Device::GyroMPU9250 mpu9250;
  static Espfc::Device::GyroLSM6DSO lsm6dso;
  static Espfc::Device::GyroICM20602 icm20602;
  static Espfc::Device::GyroBMI160 bmi160;
  static Espfc::Device::MagHMC5338L hmc5883l;
  static Espfc::Device::MagQMC5338L qmc5883l;
  static Espfc::Device::MagQMC5338P qmc5883p;
  static Espfc::Device::MagAK8963 ak8963;
  static Espfc::Device::BaroBMP085 bmp085;
  static Espfc::Device::BaroBMP280 bmp280;
  static Espfc::Device::BaroSPL06 spl06;
}

namespace Espfc {

Hardware::Hardware(Model& model): _model(model) {}

int Hardware::begin()
{
  initBus();
  detectGyro();
  detectMag();
  detectBaro();
  return 1;
}

void Hardware::onI2CError()
{
  _model.state.i2cErrorCount++;
  _model.state.i2cErrorDelta++;
}

void Hardware::initBus()
{
#if defined(ESPFC_SPI_0)
  int spiResult = spiBus.begin(_model.config.pin[PIN_SPI_0_SCK], _model.config.pin[PIN_SPI_0_MOSI], _model.config.pin[PIN_SPI_0_MISO]);
  _model.logger.info().log(F("SPI")).log(_model.config.pin[PIN_SPI_0_SCK]).log(_model.config.pin[PIN_SPI_0_MOSI]).log(_model.config.pin[PIN_SPI_0_MISO]).logln(spiResult);
#endif
#if defined(ESPFC_I2C_0)
  int i2cResult = i2cBus.begin(_model.config.pin[PIN_I2C_0_SDA], _model.config.pin[PIN_I2C_0_SCL], _model.config.i2cSpeed * 1000ul);
  i2cBus.onError = std::bind(&Hardware::onI2CError, this);
  _model.logger.info().log(F("I2C")).log(_model.config.pin[PIN_I2C_0_SDA]).log(_model.config.pin[PIN_I2C_0_SCL]).log(_model.config.i2cSpeed).logln(i2cResult);
#endif
}

void Hardware::detectGyro()
{
  if(_model.config.gyro.dev == GYRO_NONE) return;

  Device::GyroDevice * detectedGyro = nullptr;
#if defined(ESPFC_SPI_0)
  if(_model.config.pin[PIN_SPI_CS0] != -1)
  {
    Hal::Gpio::digitalWrite(_model.config.pin[PIN_SPI_CS0], HIGH);
    Hal::Gpio::pinMode(_model.config.pin[PIN_SPI_CS0], OUTPUT);
    if(!detectedGyro && detectDevice(mpu9250, spiBus, _model.config.pin[PIN_SPI_CS0])) detectedGyro = &mpu9250;
    if(!detectedGyro && detectDevice(mpu6500, spiBus, _model.config.pin[PIN_SPI_CS0])) detectedGyro = &mpu6500;
    if(!detectedGyro && detectDevice(icm20602, spiBus, _model.config.pin[PIN_SPI_CS0])) detectedGyro = &icm20602;
    if(!detectedGyro && detectDevice(bmi160, spiBus, _model.config.pin[PIN_SPI_CS0])) detectedGyro = &bmi160;
    if(!detectedGyro && detectDevice(lsm6dso, spiBus, _model.config.pin[PIN_SPI_CS0])) detectedGyro = &lsm6dso;
    if(detectedGyro) gyroSlaveBus.begin(&spiBus, detectedGyro->getAddress());
  }
#endif
#if defined(ESPFC_I2C_0)
  if(!detectedGyro && _model.config.pin[PIN_I2C_0_SDA] != -1 && _model.config.pin[PIN_I2C_0_SCL] != -1)
  {
    if(!detectedGyro && detectDevice(mpu9250, i2cBus)) detectedGyro = &mpu9250;
    if(!detectedGyro && detectDevice(mpu6500, i2cBus)) detectedGyro = &mpu6500;
    if(!detectedGyro && detectDevice(icm20602, i2cBus)) detectedGyro = &icm20602;
    if(!detectedGyro && detectDevice(bmi160, i2cBus)) detectedGyro = &bmi160;
    if(!detectedGyro && detectDevice(mpu6050, i2cBus)) detectedGyro = &mpu6050;
    if(!detectedGyro && detectDevice(lsm6dso, i2cBus)) detectedGyro = &lsm6dso;
    if(detectedGyro) gyroSlaveBus.begin(&i2cBus, detectedGyro->getAddress());
  }
#endif
  if(!detectedGyro) return;

  detectedGyro->setDLPFMode(_model.config.gyro.dlpf);
  _model.state.gyro.dev = detectedGyro;
  _model.state.gyro.present = (bool)detectedGyro;
  _model.state.accel.present = _model.state.gyro.present && _model.config.accel.dev != GYRO_NONE;
  _model.state.gyro.clock = detectedGyro->getRate();
}

void Hardware::detectMag()
{
  if(_model.config.mag.dev == MAG_NONE) return;

  Device::MagDevice * detectedMag  = nullptr;
#if defined(ESPFC_I2C_0)
  if(_model.config.pin[PIN_I2C_0_SDA] != -1 && _model.config.pin[PIN_I2C_0_SCL] != -1)
  {
    if(!detectedMag && detectDevice(ak8963, i2cBus)) detectedMag = &ak8963;
    if(!detectedMag && detectDevice(hmc5883l, i2cBus)) detectedMag = &hmc5883l;
    if(!detectedMag && detectDevice(qmc5883l, i2cBus)) detectedMag = &qmc5883l;
    if(!detectedMag && detectDevice(qmc5883p, i2cBus)) detectedMag = &qmc5883p;
  }
#endif
  if(gyroSlaveBus.getBus())
  {
    if(!detectedMag && detectDevice(ak8963, gyroSlaveBus)) detectedMag = &ak8963;
    if(!detectedMag && detectDevice(hmc5883l, gyroSlaveBus)) detectedMag = &hmc5883l;
    if(!detectedMag && detectDevice(qmc5883l, gyroSlaveBus)) detectedMag = &qmc5883l;
    if(!detectedMag && detectDevice(qmc5883p, gyroSlaveBus)) detectedMag = &qmc5883p;
    
  }
  _model.state.mag.dev = detectedMag;
  _model.state.mag.present = (bool)detectedMag;
  _model.state.mag.rate = detectedMag ? detectedMag->getRate() : 0;
}

void Hardware::detectBaro()
{
  if(_model.config.baro.dev == BARO_NONE) return;

  Device::BaroDevice * detectedBaro = nullptr;
#if defined(ESPFC_SPI_0)
  if(_model.config.pin[PIN_SPI_CS1] != -1)
  {
    Hal::Gpio::digitalWrite(_model.config.pin[PIN_SPI_CS1], HIGH);
    Hal::Gpio::pinMode(_model.config.pin[PIN_SPI_CS1], OUTPUT);
    if(!detectedBaro && detectDevice(bmp280, spiBus, _model.config.pin[PIN_SPI_CS1])) detectedBaro = &bmp280;
    if(!detectedBaro && detectDevice(bmp085, spiBus, _model.config.pin[PIN_SPI_CS1])) detectedBaro = &bmp085;
    if(!detectedBaro && detectDevice(spl06, spiBus, _model.config.pin[PIN_SPI_CS1])) detectedBaro = &spl06;
  }
#endif
#if defined(ESPFC_I2C_0)
  if(_model.config.pin[PIN_I2C_0_SDA] != -1 && _model.config.pin[PIN_I2C_0_SCL] != -1)
  {
    if(!detectedBaro && detectDevice(bmp280, i2cBus)) detectedBaro = &bmp280;
    if(!detectedBaro && detectDevice(bmp085, i2cBus)) detectedBaro = &bmp085;
    if(!detectedBaro && detectDevice(spl06, i2cBus)) detectedBaro = &spl06;
  }
#endif
  if(gyroSlaveBus.getBus())
  {
    if(!detectedBaro && detectDevice(bmp280, gyroSlaveBus)) detectedBaro = &bmp280;
    if(!detectedBaro && detectDevice(bmp085, gyroSlaveBus)) detectedBaro = &bmp085;
    if(!detectedBaro && detectDevice(spl06, gyroSlaveBus)) detectedBaro = &spl06;
  }

  _model.state.baro.dev = detectedBaro;
  _model.state.baro.present = (bool)detectedBaro;
}

void Hardware::restart(const Model& model)
{
  if(model.state.mixer.escMotor) model.state.mixer.escMotor->end();
  if(model.state.mixer.escServo) model.state.mixer.escServo->end();
#ifdef ESPFC_SERIAL_SOFT_0_WIFI
  WiFi.disconnect();
  WiFi.softAPdisconnect();
#endif
  delay(100);
  targetReset();
}

}
