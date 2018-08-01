#ifndef _ESPFC_HARDWARE_H_
#define _ESPFC_HARDWARE_H_

#include <Arduino.h>
#include "Model.h"
#include "EspSoftSerial.h"
#include "SerialDeviceAdapter.h"
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
#include "Device/BaroDevice.h"
#include "Device/BaroBMP085.h"

namespace {
#if defined(ESP32)
  static HardwareSerial Serial1(1);
  static HardwareSerial Serial2(2);
  static Espfc::SerialDeviceAdapter<HardwareSerial> uart0(Serial);
  static Espfc::SerialDeviceAdapter<HardwareSerial> uart1(Serial1);
  static Espfc::SerialDeviceAdapter<HardwareSerial> uart2(Serial2);
#endif

#if defined(ESP8266)
  static Espfc::SerialDeviceAdapter<HardwareSerial> uart0(Serial);
  static Espfc::SerialDeviceAdapter<HardwareSerial> uart1(Serial1);
#if defined(USE_SOFT_SERIAL)
  static EspSoftSerial softSerial;
  static Espfc::SerialDeviceAdapter<EspSoftSerial>  soft0(softSerial);
#endif

#endif
  static Espfc::InputPPM ppm;
  static Espfc::InputSBUS sbus;
  static Espfc::Device::BusSPI spiBus;
  static Espfc::Device::BusI2C i2cBus;
  static Espfc::Device::GyroMPU6050 mpu6050;
  static Espfc::Device::GyroMPU9250 mpu9250;
  static Espfc::Device::MagHMC5338L hmc5883l;
  static Espfc::Device::BaroBMP085 bmp085;
  static Espfc::Device::GyroDevice * detectedGyro = nullptr;
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
      initSerial();
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
      _model.logger.info().log(F("SPI SETUP")).logln(spiResult);
#elif defined(ESP8266)
      //int spiResult = spiBus.begin();
      //_model.logger.info().log(F("SPI")).logln(spiResult);
#endif
    }

    void detectGyro()
    {
      if(_model.config.gyroDev == GYRO_NONE) return;

      _model.state.gyroDev = GYRO_NONE;
      _model.state.gyroBus = BUS_NONE;

#if defined(ESP32)
      detectGyroDevice(mpu9250, spiBus, _model.config.pin[PIN_SPI_0_CS0]);
#endif
      detectGyroDevice(mpu9250, i2cBus);
      detectGyroDevice(mpu6050, i2cBus);

      _model.state.gyroPresent = (bool)detectedGyro;
      _model.state.accelPresent = _model.state.gyroPresent && _model.config.accelDev != GYRO_NONE;
    }

    void detectGyroDevice(Device::GyroDevice& gyro, Device::BusSPI& bus, int cs)
    {
      if(detectedGyro) return;
      
      bool status = detectDevice(gyro, bus, cs);

      if(!status) return;

      detectedGyro = &gyro;

      _model.state.gyroBus = BUS_SPI;
      _model.state.gyroDev = gyro.getType();
    }

    void detectGyroDevice(Device::GyroDevice& gyro, Device::BusI2C& bus)
    {
      if(detectedGyro) return;

      bool status = detectDevice(gyro, bus);

      if(!status) return;

      detectedGyro = &gyro;

      _model.state.gyroBus = BUS_I2C;
      _model.state.gyroDev = gyro.getType();
    }

    template<typename Dev>
    bool detectDevice(Dev& dev, Device::BusSPI& bus, int cs)
    {
      if(cs == -1) return false;
      pinMode(cs, OUTPUT);
      digitalWrite(cs, HIGH);
      typename Dev::DeviceType type = dev.getType();
      bool status = dev.begin(&bus, cs);
      _model.logger.info().log(F("SPI DETECT")).log(FPSTR(Dev::getName(type))).logln(status);
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

    void detectMag()
    {
      _model.state.magDev = MAG_NONE;
      _model.state.magBus = BUS_NONE;
      if(_model.config.magDev == MAG_NONE) return;
      int status = detectDevice(hmc5883l, i2cBus);
      if(status)
      {
        _model.state.magDev = MAG_HMC5883;
        _model.state.magBus = BUS_I2C;
      }
      _model.state.magPresent = status;
    }

    void detectBaro()
    {
      _model.state.baroDev = BARO_NONE;
      _model.state.baroBus = BUS_NONE;
      if(_model.config.baroDev == BARO_NONE) return;
      int status = detectDevice(bmp085, i2cBus);
      if(status)
      {
        _model.state.baroDev = BARO_BMP085;
        _model.state.baroBus = BUS_I2C;
      }
      _model.state.baroPresent = status;
    }

    void initSerial()
    {
      for(int i = SERIAL_UART_0; i < SERIAL_UART_COUNT; i++)
      {
        SerialDevice * port = getSerialPortById((SerialPort)i);
        if(!port) continue;

        const SerialPortConfig& spc = _model.config.serial[i];
        SerialDeviceConfig sdc;

#if defined(ESP32)
        sdc.tx_pin = _model.config.pin[i * 2 + PIN_SERIAL_0_TX];
        sdc.rx_pin = _model.config.pin[i * 2 + PIN_SERIAL_0_RX];
#endif

        sdc.baud = fromIndex((SerialSpeedIndex)spc.baudIndex, SERIAL_SPEED_115200);

        if(spc.functionMask & SERIAL_FUNCTION_RX_SERIAL)
        {
          switch(_model.config.input.serialRxProvider)
          {
            case SERIALRX_SBUS:
              sdc.baud = 100000;
              sdc.parity = SERIAL_PARITY_EVEN;
              sdc.stop_bits = SERIAL_STOP_BITS_2;
              sdc.inverted = true;
              break;
            default:
              break;
          }
        }

        _model.logger.info().log(F("UART")).log(i).log(spc.id).log(spc.functionMask).log(sdc.baud).log(sdc.tx_pin).logln(sdc.rx_pin);
        port->flush();
        delay(20);
        port->begin(sdc);
        _model.state.serial[i].stream = port;
      }       
    }

    static SerialDevice * getSerialPortById(SerialPort portId)
    {
      switch(portId)
      {
        case SERIAL_UART_0: return &uart0;
        case SERIAL_UART_1: return &uart1;
#if defined(ESP32)
        case SERIAL_UART_2: return &uart2;
#elif defined(ESP8266) && defined(USE_SOFT_SERIAL)
        case SERIAL_SOFT_0: return &soft0;
#endif
        default: return nullptr;
      }
    }

    SerialSpeed fromIndex(SerialSpeedIndex index, SerialSpeed defaultSpeed)
    {
      switch(index)
      {
        case SERIAL_SPEED_INDEX_9600:   return SERIAL_SPEED_9600;
        case SERIAL_SPEED_INDEX_19200:  return SERIAL_SPEED_19200;
        case SERIAL_SPEED_INDEX_38400:  return SERIAL_SPEED_38400;
        case SERIAL_SPEED_INDEX_57600:  return SERIAL_SPEED_57600;
        case SERIAL_SPEED_INDEX_115200: return SERIAL_SPEED_115200;
        case SERIAL_SPEED_INDEX_230400: return SERIAL_SPEED_230400;
        case SERIAL_SPEED_INDEX_250000: return SERIAL_SPEED_250000;
        case SERIAL_SPEED_INDEX_500000: return SERIAL_SPEED_500000;
        case SERIAL_SPEED_INDEX_AUTO:
        default:
          return defaultSpeed;
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
      if(model.config.magDev == BARO_NONE) return nullptr;
      return &hmc5883l;
    }

    static Device::BaroDevice * getBaroDevice(const Model& model)
    {
      if(model.config.baroDev == BARO_NONE) return nullptr;
      return &bmp085;
    }

    static InputDevice * getInputDevice(Model& model)
    {
      SerialDevice * serial = model.getSerialStream(SERIAL_FUNCTION_RX_SERIAL);
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
      #if defined(ESP8266)
        escMotor.begin((EscProtocol)_model.config.output.protocol, _model.config.output.async, _model.config.output.rate, ESC_DRIVER_TIMER1);
        _model.logger.info().log(F("MOTOR CONF")).log(ESC_DRIVER_TIMER1).log(_model.config.output.protocol).log(_model.config.output.async).logln(_model.config.output.rate);
        escServo.begin(ESC_PROTOCOL_PWM, true, _model.config.output.servoRate, ESC_DRIVER_TIMER2);
        _model.logger.info().log(F("SERVO CONF")).log(ESC_DRIVER_TIMER2).log(ESC_PROTOCOL_PWM).log(true).logln(_model.config.output.servoRate);
      #elif defined(ESP32)
        escMotor.begin((EscProtocol)_model.config.output.protocol, _model.config.output.async, _model.config.output.rate);
        _model.logger.info().log(F("MOTOR CONF")).log(_model.config.output.protocol).log(_model.config.output.async).logln(_model.config.output.rate);
        escServo.begin(ESC_PROTOCOL_PWM, true, _model.config.output.servoRate);
        _model.logger.info().log(F("SERVO CONF")).log(ESC_PROTOCOL_PWM).log(true).logln(_model.config.output.servoRate);
      #endif

      for(size_t i = 0; i < OUTPUT_CHANNELS; ++i)
      {
        const OutputChannelConfig& occ = _model.config.output.channel[i];
        if(occ.servo)
        {
          escServo.attach(i, _model.config.pin[PIN_OUTPUT_0 + i], 1500);
          _model.logger.info().log(F("SERVO PIN")).log(i).logln(_model.config.pin[PIN_OUTPUT_0 + i]);
        }
        else
        {
          escMotor.attach(i, _model.config.pin[PIN_OUTPUT_0 + i], 1000);
          _model.logger.info().log(F("MOTOR PIN")).log(i).logln(_model.config.pin[PIN_OUTPUT_0 + i]);
        }
      }
    }

    static EscDriver * getMotorDriver()
    {
      return &escMotor;
    }

    static EscDriver * getServoDriver()
    {
      return &escServo;
    }

    static void restart(Model& model)
    {
      getMotorDriver()->end();
      getServoDriver()->end();

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
