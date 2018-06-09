#ifndef _ESPFC_SERIAL_H_
#define _ESPFC_SERIAL_H_

#include <Arduino.h>

#if defined(ESP8266)
#include <ESP8266WiFi.h>
#endif

#include "Model.h"
#include "EspSoftSerial.h"
#include "SerialDevice.h"
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

namespace {
  static Espfc::Device::BusSPI spiBus;
  static Espfc::Device::BusI2C i2cBus;
  static Espfc::Device::GyroMPU6050 mpu6050;
  static Espfc::Device::GyroMPU9250 mpu9250;
  static Espfc::Device::MagHMC5338L hmc5883l;
  static Espfc::Device::GyroDevice * detectedGyro = nullptr;
}

namespace Espfc {

class Hardware
{
  public:
    Hardware(Model& model): _model(model) {}

    int begin()
    {
      initSerial();
      initWifi();
      initBus();
      detectGyro();
      detectMag();
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
      if(cs == -1) return;

      int type = gyro.getType();

      pinMode(cs, OUTPUT);
      digitalWrite(cs, HIGH);
      bool status = gyro.begin(&bus, cs);

      _model.logger.info().log(F("SPI DETECT")).log(type).logln(status);

      if(!status) return;

      detectedGyro = &gyro;

      _model.state.gyroBus = BUS_SPI;
      _model.state.gyroDev = type;
    }

    void detectGyroDevice(Device::GyroDevice& gyro, Device::BusI2C& bus)
    {
      if(detectedGyro) return;

      int type = gyro.getType();
      bool status = gyro.begin(&bus);

      _model.logger.info().log(F("I2C DETECT")).log(type).logln(status);

      if(!status) return;

      detectedGyro = &gyro;

      _model.state.gyroBus = BUS_I2C;
      _model.state.gyroDev = type;
    }

    void detectMag()
    {
      if(_model.config.magDev != MAG_NONE)
      {
        _model.state.magPresent = hmc5883l.begin(&i2cBus);
      }
    }

    void initWifi()
    {
#if defined(ESP8266)
      WiFi.disconnect();
      WiFi.mode(WIFI_OFF);
      _model.logger.info().logln(F("WIFI OFF"));
#endif
    }

    void initSerial()
    {
      for(int i = SERIAL_UART_0; i < SERIAL_UART_COUNT; i++)
      {
        SerialDevice * serial = getSerialPortById((SerialPort)i);

        if(!serial) continue;
        if(!_model.isActive(FEATURE_SOFTSERIAL) && _model.config.serial[i].id >= 30) continue;

        bool bbx = _model.config.serial[i].functionMask & SERIAL_FUNCTION_BLACKBOX;
        bool msp = _model.config.serial[i].functionMask & SERIAL_FUNCTION_MSP;
        bool deb = _model.config.serial[i].functionMask & SERIAL_FUNCTION_TELEMETRY_FRSKY;
        bool srx = _model.config.serial[i].functionMask & SERIAL_FUNCTION_RX_SERIAL;

        SerialDeviceConfig sc;

    #if defined(ESP32)
        sc.tx_pin = _model.config.pin[i * 2 + PIN_SERIAL_0_TX];
        sc.rx_pin = _model.config.pin[i * 2 + PIN_SERIAL_0_RX];
    #endif

        if(srx)
        {
          sc.baud = 100000; // sbus
          sc.stop_bits = SERIAL_STOP_BITS_2;
          sc.parity = SERIAL_PARITY_EVEN;
          sc.inverted = true;
          sc.rx_pin = _model.config.pin[PIN_INPUT_RX];
          serial->flush();
          serial->begin(sc);
          _model.logger.info().log(F("UART")).log(i).log(sc.baud).log(sc.inverted).logln(F("sbus"));
        }
        else if(bbx && msp)
        {
          int idx = _model.config.serial[i].blackboxBaudIndex == SERIAL_SPEED_INDEX_AUTO ? _model.config.serial[i].baudIndex : _model.config.serial[i].blackboxBaudIndex;
          sc.baud = fromIndex((SerialSpeedIndex)idx, SERIAL_SPEED_115200);
          serial->flush();
          serial->begin(sc);
          _model.logger.info().log(F("UART")).log(i).log(sc.baud).log(sc.inverted).log(F("msp")).logln(F("blackbox"));
        }
        else if(bbx)
        {
          int idx = _model.config.serial[i].blackboxBaudIndex == SERIAL_SPEED_INDEX_AUTO ? _model.config.serial[i].baudIndex : _model.config.serial[i].blackboxBaudIndex;
          sc.baud = fromIndex((SerialSpeedIndex)idx, SERIAL_SPEED_250000);
          serial->flush();
          serial->begin(sc);
          _model.logger.info().log(F("UART")).log(i).log(sc.baud).log(sc.inverted).logln(F("blackbox"));
        }
        else if(msp || deb)
        {
          sc.baud = fromIndex((SerialSpeedIndex)_model.config.serial[i].baudIndex, SERIAL_SPEED_115200);
          serial->flush();
          serial->begin(sc);
          _model.logger.info().log(F("UART")).log(i).log(sc.baud).log(msp ? F("msp") : F("")).logln(deb ? F("debug") : F(""));
        }
        else
        {
          serial->flush();
          _model.logger.info().log(F("UART")).log(i).logln(F("free"));
        }
      }
    }

    static SerialDevice * getSerialPort(SerialPortConfig * config, SerialFunction sf)
    {
      for (int i = SERIAL_UART_0; i < SERIAL_UART_COUNT; i++)
      {
        if(config[i].functionMask & sf) return getSerialPortById((SerialPort)i);
      }
      return NULL;
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

    static SerialDevice * getSerialPortById(SerialPort portId)
    {
#if defined(ESP32)
      static HardwareSerial Serial1(1);
      static HardwareSerial Serial2(2);
      static SerialDeviceAdapter<HardwareSerial> uart0(Serial);
      static SerialDeviceAdapter<HardwareSerial> uart1(Serial1);
      static SerialDeviceAdapter<HardwareSerial> uart2(Serial2);
#endif
#if defined(ESP8266)
  #if defined(USE_SOFT_SERIAL)
      static EspSoftSerial softSerial;
      static SerialDeviceAdapter<EspSoftSerial>  soft0(softSerial);
  #endif
      static SerialDeviceAdapter<HardwareSerial> uart0(Serial);
      static SerialDeviceAdapter<HardwareSerial> uart1(Serial1);
#endif

      switch(portId)
      {
        case SERIAL_UART_0: return &uart0;
        case SERIAL_UART_1: return &uart1;
#if defined(ESP32)
        case SERIAL_UART_2: return &uart2;
#elif defined(ESP8266) && defined(USE_SOFT_SERIAL)
        case SERIAL_SOFT_0: return &soft0;
#endif
        default: return NULL;
      }
    }

    static Device::GyroDevice * getGyroDevice(const Model& model)
    {
      return &mpu6050;
      //return detectedGyro;
    }

    static Device::MagDevice * getMagDevice(const Model& model)
    {
      return &hmc5883l;
    }

    static InputDevice * getInputDevice(Model& model)
    {
      static InputPPM ppm;
      static InputSBUS sbus;

      SerialDevice * serial = getSerialPort(model.config.serial, SERIAL_FUNCTION_RX_SERIAL);
      if(serial)
      {
        sbus.begin(serial);
        model.logger.info().log(F("SBUS RX")).logln(model.config.pin[PIN_INPUT_RX]);
        return &sbus;
      }
      else if(model.isActive(FEATURE_RX_PPM) && model.config.pin[PIN_INPUT_RX] != -1)
      {
        ppm.begin(model.config.pin[PIN_INPUT_RX], model.config.input.ppmMode);
        model.logger.info().log(F("PPM RX")).log(model.config.pin[PIN_INPUT_RX]).logln(model.config.input.ppmMode);
        return &ppm;
      }
      return NULL;
    }

    static EscDriver * getEscDriver(Model& model)
    {
      static EscDriver driver;

      driver.begin((EscProtocol)model.config.output.protocol, model.config.output.async, model.config.output.rate);
      model.logger.info().log(F("OUTPUT CONF")).log(model.config.output.protocol).log(model.config.output.rate).logln(model.config.output.async);

      for(size_t i = 0; i < OUTPUT_CHANNELS; ++i)
      {
        driver.attach(i, model.config.pin[PIN_OUTPUT_0 + i], model.state.outputDisarmed[i]);
        model.logger.info().log(F("OUTPUT PIN")).log(i).logln(model.config.pin[PIN_OUTPUT_0 + i]);
      }

      return &driver;
    }

    static void restart(Model& model)
    {
      getEscDriver(model)->end();

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
