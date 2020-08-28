#ifndef _ESPFC_HARDWARE_H_
#define _ESPFC_HARDWARE_H_

#include <Arduino.h>
#include "Model.h"
#include "EspSoftSerial.h"
#include "InputDevice.h"
#include "InputPPM.h"
#include "InputSBUS.h"
#include "EscDriver.h"
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
#include "Device/SerialDevice.h"
#include "Device/SerialDeviceAdapter.h"

namespace {
#if defined(ESP32)
  static HardwareSerial Serial1(1);
  static HardwareSerial Serial2(2);
  static Espfc::Device::SerialDeviceAdapter<HardwareSerial> uart0(Serial);
  static Espfc::Device::SerialDeviceAdapter<HardwareSerial> uart1(Serial1);
  static Espfc::Device::SerialDeviceAdapter<HardwareSerial> uart2(Serial2);
#endif

#if defined(ESP8266)
  static Espfc::Device::SerialDeviceAdapter<HardwareSerial> uart0(Serial);
  static Espfc::Device::SerialDeviceAdapter<HardwareSerial> uart1(Serial1);
#if defined(USE_SOFT_SERIAL)
  static EspSoftSerial softSerial;
  static Espfc::Device::SerialDeviceAdapter<EspSoftSerial>  soft0(softSerial);
#endif

#endif
  static Espfc::InputPPM ppm;
  static Espfc::InputSBUS sbus;
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
  static EscDriver escMotor;
  static EscDriver escServo;
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
      initEscDrivers();
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

    static Device::SerialDevice * getSerialPortById(SerialPort portId)
    {
      switch(portId)
      {
        case SERIAL_UART_0: return &uart0;
        case SERIAL_UART_1: return &uart1;
#if defined(ESP32)
        case SERIAL_UART_2: return &uart2;
#elif defined(USE_SOFT_SERIAL)
        case SERIAL_SOFT_0: return &soft0;
#endif
        default: return nullptr;
      }
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

    static InputDevice * getInputDevice(Model& model)
    {
      Device::SerialDevice * serial = model.getSerialStream(SERIAL_FUNCTION_RX_SERIAL);
      if(serial && model.isActive(FEATURE_RX_SERIAL) && model.config.input.serialRxProvider == SERIALRX_SBUS)
      {
        sbus.begin(serial);
        model.logger.info().logln(F("RX SBUS"));
        return &sbus;
      }
      else if(model.isActive(FEATURE_RX_PPM) && model.config.pin[PIN_INPUT_RX] != -1)
      {
        ppm.begin(model.config.pin[PIN_INPUT_RX], model.config.input.ppmMode);
        model.logger.info().log(F("RX PPM")).log(model.config.pin[PIN_INPUT_RX]).logln(model.config.input.ppmMode);
        return &ppm;
      }
      return nullptr;
    }

    void initEscDrivers()
    {
      escMotor.begin((EscProtocol)_model.config.output.protocol, _model.config.output.async, _model.config.output.rate, ESC_DRIVER_MOTOR_TIMER);
      _model.logger.info().log(F("MOTOR CONF")).log(_model.config.output.protocol).log(_model.config.output.async).log(_model.config.output.rate).logln(ESC_DRIVER_MOTOR_TIMER);
      if(_model.config.output.servoRate)
      {
        escServo.begin(ESC_PROTOCOL_PWM, true, _model.config.output.servoRate, ESC_DRIVER_SERVO_TIMER);
        _model.logger.info().log(F("SERVO CONF")).log(ESC_PROTOCOL_PWM).log(true).logln(_model.config.output.servoRate).logln(ESC_DRIVER_SERVO_TIMER);
      }

      for(size_t i = 0; i < OUTPUT_CHANNELS; ++i)
      {
        const OutputChannelConfig& occ = _model.config.output.channel[i];
        if(occ.servo)
        {
          if(_model.config.output.servoRate)
          {
            escServo.attach(i, _model.config.pin[PIN_OUTPUT_0 + i], 1500);
            _model.logger.info().log(F("SERVO PIN")).log(i).logln(_model.config.pin[PIN_OUTPUT_0 + i]);
          }
        }
        else
        {
          escMotor.attach(i, _model.config.pin[PIN_OUTPUT_0 + i], 1000);
          _model.logger.info().log(F("MOTOR PIN")).log(i).logln(_model.config.pin[PIN_OUTPUT_0 + i]);
        }
      }
    }

    static EscDriver * getMotorDriver(const Model& model)
    {
      return &escMotor;
    }

    static EscDriver * getServoDriver(const Model& model)
    {
      if(model.config.output.servoRate) return &escServo;
      return nullptr;
    }

    static void restart(const Model& model)
    {
      escMotor.end();
      if(model.config.output.servoRate) escServo.end();

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
