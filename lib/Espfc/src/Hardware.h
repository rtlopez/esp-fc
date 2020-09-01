#ifndef _ESPFC_HARDWARE_H_
#define _ESPFC_HARDWARE_H_

#include <Arduino.h>
#include "Model.h"
#include "Device/BusDevice.h"
#include "Device/BusI2C.h"
#include "Device/BusSPI.h"
#include "Device/GyroDevice.h"
#include "Device/GyroMPU6050.h"
#include "Device/GyroMPU9250.h"
#include "Device/MagHMC5338L.h"
#include "Device/MagAK8963.h"
#include "Device/BaroDevice.h"
#include "Device/BaroBMP085.h"
#include "Device/BaroBMP280.h"

namespace {
  static Espfc::Device::BusSPI spiBus;
  static Espfc::Device::BusI2C i2cBus;
  static Espfc::Device::GyroMPU6050 mpu6050;
  static Espfc::Device::GyroMPU9250 mpu9250;
  static Espfc::Device::MagHMC5338L hmc5883l;
  static Espfc::Device::MagAK8963 ak8963;
  static Espfc::Device::BaroBMP085 bmp085;
  static Espfc::Device::BaroBMP280 bmp280;
  static Espfc::Device::GyroDevice * detectedGyro = nullptr;
  static Espfc::Device::BaroDevice * detectedBaro = nullptr;
  static Espfc::Device::MagDevice  * detectedMag  = nullptr;
}

namespace Espfc {

class Hardware
{
  public:
    Hardware(Model& model): _model(model) {}

    int begin()
    {
      initBus();
      detectGyro();
      detectMag();
      detectBaro();
      return 1;
    }

    void onI2CError()
    {
      _model.state.i2cErrorCount++;
      _model.state.i2cErrorDelta++;
    }

    void initBus()
    {
      int i2cResult = i2cBus.begin(_model.config.pin[PIN_I2C_0_SDA], _model.config.pin[PIN_I2C_0_SCL], _model.config.i2cSpeed * 1000);
      i2cBus.onError = std::bind(&Hardware::onI2CError, this);
      _model.logger.info().log(F("I2C SETUP")).log(_model.config.i2cSpeed).logln(i2cResult);

#if defined(ESP32)
      int spiResult = spiBus.begin(_model.config.pin[PIN_SPI_0_SCK], _model.config.pin[PIN_SPI_0_MISO], _model.config.pin[PIN_SPI_0_MOSI]);
      _model.logger.info().log(F("SPI SETUP")).log(_model.config.pin[PIN_SPI_0_SCK]).log(_model.config.pin[PIN_SPI_0_MISO]).log(_model.config.pin[PIN_SPI_0_MOSI]).logln(spiResult);
#elif defined(ESP8266)
      //int spiResult = spiBus.begin();
      //_model.logger.info().log(F("SPI")).logln(spiResult);
#endif
    }

    void detectGyro()
    {
      if(_model.config.gyroDev == GYRO_NONE) return;

#if defined(ESP32)
      if(_model.config.pin[PIN_SPI_CS0] != -1)
      {
        pinMode(_model.config.pin[PIN_SPI_CS0], OUTPUT);
        digitalWrite(_model.config.pin[PIN_SPI_CS0], HIGH);
        if(!detectedGyro && detectDevice(mpu9250, spiBus, _model.config.pin[PIN_SPI_CS0])) detectedGyro = &mpu9250;
      }
#endif
      if(_model.config.pin[PIN_I2C_0_SDA] != -1 && _model.config.pin[PIN_I2C_0_SCL] != -1)
      {
        if(!detectedGyro && detectDevice(mpu9250, i2cBus)) detectedGyro = &mpu9250;
        if(!detectedGyro && detectDevice(mpu6050, i2cBus)) detectedGyro = &mpu6050;
      }
      _model.state.gyroPresent = (bool)detectedGyro;
      _model.state.accelPresent = _model.state.gyroPresent && _model.config.accelDev != GYRO_NONE;
    }

    void detectMag()
    {
      if(_model.config.magDev == MAG_NONE) return;

#if defined(ESP32)
      if(_model.config.pin[PIN_SPI_CS0] != -1 && detectedGyro && detectedGyro->getType() == GYRO_MPU9250)
      {
        if(!detectedMag && detectDevice(ak8963, spiBus, _model.config.pin[PIN_SPI_CS0])) detectedMag = &ak8963;
      }
#endif
      if(_model.config.pin[PIN_I2C_0_SDA] != -1 && _model.config.pin[PIN_I2C_0_SCL] != -1)
      {
        if(detectedGyro && detectedGyro->getType() == GYRO_MPU9250)
        {
          if(!detectedMag && detectDevice(ak8963, i2cBus)) detectedMag = &ak8963;
        }
        if(!detectedMag && detectDevice(hmc5883l, i2cBus)) detectedMag = &hmc5883l;
      }
      _model.state.magPresent = (bool)detectedMag;
      _model.state.magRate = detectedMag ? detectedMag->getRate() : 0;
    }

    void detectBaro()
    {
      if(_model.config.baroDev == BARO_NONE) return;

#if defined(ESP32)
      if(_model.config.pin[PIN_SPI_CS1] != -1)
      {
        pinMode(_model.config.pin[PIN_SPI_CS1], OUTPUT);
        digitalWrite(_model.config.pin[PIN_SPI_CS1], HIGH);
        if(!detectedBaro && detectDevice(bmp280, spiBus, _model.config.pin[PIN_SPI_CS1])) detectedBaro = &bmp280;
        if(!detectedBaro && detectDevice(bmp085, spiBus, _model.config.pin[PIN_SPI_CS1])) detectedBaro = &bmp085;
      }
#endif
      if(_model.config.pin[PIN_I2C_0_SDA] != -1 && _model.config.pin[PIN_I2C_0_SCL] != -1)
      {
        if(!detectedBaro && detectDevice(bmp280, i2cBus)) detectedBaro = &bmp280;
        if(!detectedBaro && detectDevice(bmp085, i2cBus)) detectedBaro = &bmp085;
      }
      _model.state.baroPresent = (bool)detectedBaro;
    }

    template<typename Dev>
    bool detectDevice(Dev& dev, Device::BusSPI& bus, int cs)
    {
      typename Dev::DeviceType type = dev.getType();
      bool status = dev.begin(&bus, cs);
      _model.logger.info().log(F("SPI DETECT")).log(FPSTR(Dev::getName(type))).log(cs).logln(status);
      return status;
    }

    template<typename Dev>
    bool detectDevice(Dev& dev, Device::BusI2C& bus)
    {
      typename Dev::DeviceType type = dev.getType();
      bool status = dev.begin(&bus);
      _model.logger.info().log(F("I2C DETECT")).log(FPSTR(Dev::getName(type))).logln(status);
      return status;
    }

    int update()
    {
      return 1;
    }

    static Device::GyroDevice * getGyroDevice(const Model& model)
    {
      return detectedGyro;
    }

    static Device::MagDevice * getMagDevice(const Model& model)
    {
      if(model.config.magDev == MAG_NONE) return nullptr;
      return detectedMag;
    }

    static Device::BaroDevice * getBaroDevice(const Model& model)
    {
      if(model.config.baroDev == BARO_NONE) return nullptr;
      return detectedBaro;
    }

    static void restart(const Model& model)
    {
      //escMotor.end();
      //if(model.config.output.servoRate) escServo.end();

#if defined(ESP8266)
      // pin setup to ensure boot from flash
      pinMode(0, OUTPUT); digitalWrite(0, HIGH); // GPIO0 to HI
      pinMode(2, OUTPUT); digitalWrite(2, HIGH); // GPIO2 to HI
      pinMode(15, OUTPUT); digitalWrite(15, LOW); // GPIO15 to LO
      pinMode(0, INPUT);
      pinMode(2, INPUT);
      pinMode(15, INPUT);
      //Serial.println("hard reset");
      ESP.reset();
#elif defined(ESP32)
      //Serial.println("normal restart");
      ESP.restart();
#endif

      while(1) {}
    }

  private:
    Model& _model;
};

}

#endif
