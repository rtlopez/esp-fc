#ifndef _ESPFC_SERIAL_H_
#define _ESPFC_SERIAL_H_

#include <Wire.h>
#include "Arduino.h"
#include "Model.h"
#include "EspSoftSerial.h"
#include "SerialDevice.h"
#include "InputDevice.h"
#include "InputPPM.h"
#include "InputSBUS.h"

#if defined(ESP32)
HardwareSerial Serial1(1);
HardwareSerial Serial2(1);
#endif

namespace Espfc {

class Hardware
{
  public:
    Hardware(Model& model): _model(model) {}

    int begin()
    {
      Wire.begin(_model.config.pin[PIN_I2C_0_SDA], _model.config.pin[PIN_I2C_0_SCL]);
      Wire.setClock(400000);
      //Wire.setClock(600000);
      //Wire.setClock(1000000); // in real ~640kHz on ESP8266

      _model.logger.info().logln(F("I2C"));

      for(int i = SERIAL_UART_0; i < SERIAL_UART_COUNT; i++)
      {
        SerialDevice * serial = getSerialPortById((SerialPort)i);
        if(!serial) continue;

        bool bbx = _model.config.serial[i].functionMask & SERIAL_FUNCTION_BLACKBOX;
        bool msp = _model.config.serial[i].functionMask & SERIAL_FUNCTION_MSP;
        bool deb = _model.config.serial[i].functionMask & SERIAL_FUNCTION_TELEMETRY_FRSKY;
        bool srx = _model.config.serial[i].functionMask & SERIAL_FUNCTION_RX_SERIAL;

        SerialDeviceConfig sc;
        if(srx)
        {
          sc.baud = 100000; // sbus
          sc.inverted = true;
          //sc.rx_pin = _model.config.inputPin;
          serial->begin(sc);
          _model.logger.info().log(F("UART")).log(i).log(sc.baud).log(sc.inverted).logln(F("sbus"));
        }
        else if(bbx && msp)
        {
          int idx = _model.config.serial[i].blackboxBaudIndex == SERIAL_SPEED_INDEX_AUTO ? _model.config.serial[i].baudIndex : _model.config.serial[i].blackboxBaudIndex;
          sc.baud = fromIndex((SerialSpeedIndex)idx, SERIAL_SPEED_115200);
          serial->begin(sc);
          _model.logger.info().log(F("UART")).log(i).log(sc.baud).log(sc.inverted).log(F("msp")).logln(F("blackbox"));
        }
        else if(bbx)
        {
          int idx = _model.config.serial[i].blackboxBaudIndex == SERIAL_SPEED_INDEX_AUTO ? _model.config.serial[i].baudIndex : _model.config.serial[i].blackboxBaudIndex;
          sc.baud = fromIndex((SerialSpeedIndex)idx, SERIAL_SPEED_250000);
          serial->begin(sc);
          _model.logger.info().log(F("UART")).log(i).log(sc.baud).log(sc.inverted).logln(F("blackbox"));
        }
        else if(msp || deb)
        {
          sc.baud = fromIndex((SerialSpeedIndex)_model.config.serial[i].baudIndex, SERIAL_SPEED_115200);
          serial->begin(sc);
          _model.logger.info().log(F("UART")).log(i).log(sc.baud).log(msp ? F("msp") : F("")).logln(deb ? F("debug") : F(""));
        }
      }
      return 1;
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
      static SerialDeviceAdapter<HardwareSerial> uart0(Serial);
      static SerialDeviceAdapter<HardwareSerial> uart1(Serial1);
#if defined(ESP32)
      static SerialDeviceAdapter<HardwareSerial> uart2(Serial2);
#endif
#if defined(ESP8266)
      static EspSoftSerial softSerial;
      static SerialDeviceAdapter<EspSoftSerial>  soft0(softSerial);
#endif

      switch(portId)
      {
        case SERIAL_UART_0: return &uart0;
        case SERIAL_UART_1: return NULL; //return &uart1;
#if defined(ESP32)
        case SERIAL_UART_2: return &uart2;
#endif
#if defined(ESP8266)
        case SERIAL_SOFT_0: return &soft0;
#endif
        default: return NULL;
      }
    }

    static InputDevice * getInputDevice(Model& model)
    {
      static InputPPM ppm;
      static InputSBUS sbus;

      SerialDevice * serial = getSerialPort(model.config.serial, SERIAL_FUNCTION_RX_SERIAL);
      if(serial)
      {
        sbus.begin(serial);
        model.logger.info().logln(F("SBUS RX"));//.logln(model.config.inputPin);
        return &sbus;
      }
      else if(model.isActive(FEATURE_RX_PPM) && model.config.pin[PIN_INPUT_RX] != -1)
      {
        ppm.begin(model.config.pin[PIN_INPUT_RX], model.config.ppmMode);
        model.logger.info().log(F("PPM RX")).log(model.config.pin[PIN_INPUT_RX]).logln(model.config.ppmMode);
        return &ppm;
      }
      return NULL;
    }

  private:
    Model& _model;
};

}

#endif
