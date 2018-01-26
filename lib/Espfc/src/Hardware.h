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

namespace Espfc {

class Hardware
{
  public:
    Hardware(Model& model): _model(model) {}

    int begin()
    {
      Wire.begin(_model.config.pin[PIN_I2C_0_SDA], _model.config.pin[PIN_I2C_0_SCL]);
      //Wire.setClock(100000);
      //Wire.setClock(500000);
      //Wire.setClock(600000);
      Wire.setClock(1000000); // ~640kHz on ESP8266

      _model.logger.info().logln(F("I2C"));

      for(int i = SERIAL_UART_0; i < SERIAL_UART_COUNT; i++)
      {
        SerialDevice * serial = getSerialPortById((SerialPort)i);

        bool bbx = _model.config.serial[i].functionMask & SERIAL_FUNCTION_BLACKBOX;
        bool msp = _model.config.serial[i].functionMask & SERIAL_FUNCTION_MSP;
        bool deb = _model.config.serial[i].functionMask & SERIAL_FUNCTION_TELEMETRY_FRSKY;
        bool srx = _model.config.serial[i].functionMask & SERIAL_FUNCTION_RX_SERIAL;

        SerialDeviceConfig sc;
        sc.tx_pin = _model.config.pin[i * 2 + PIN_SERIAL_0_TX];
        sc.rx_pin = _model.config.pin[i * 2 + PIN_SERIAL_0_RX];

        if(srx)
        {
          sc.baud = 100000; // sbus
          sc.stop_bits = SERIAL_STOP_BITS_2;
          sc.inverted = true;
          if(serial)
          {
            serial->flush();
            serial->begin(sc);
          }
          _model.logger.info().log(F("UART")).log(i).log(sc.baud).log(sc.inverted).logln(F("sbus"));
        }
        else if(bbx && msp)
        {
          int idx = _model.config.serial[i].blackboxBaudIndex == SERIAL_SPEED_INDEX_AUTO ? _model.config.serial[i].baudIndex : _model.config.serial[i].blackboxBaudIndex;
          sc.baud = fromIndex((SerialSpeedIndex)idx, SERIAL_SPEED_115200);
          if(serial)
          {
            serial->flush();
            serial->begin(sc);
          }
          _model.logger.info().log(F("UART")).log(i).log(sc.baud).log(sc.inverted).log(F("msp")).logln(F("blackbox"));
        }
        else if(bbx)
        {
          int idx = _model.config.serial[i].blackboxBaudIndex == SERIAL_SPEED_INDEX_AUTO ? _model.config.serial[i].baudIndex : _model.config.serial[i].blackboxBaudIndex;
          sc.baud = fromIndex((SerialSpeedIndex)idx, SERIAL_SPEED_250000);
          if(serial)
          {
            serial->flush();
            serial->begin(sc);
          }
          _model.logger.info().log(F("UART")).log(i).log(sc.baud).log(sc.inverted).logln(F("blackbox"));
        }
        else if(msp || deb)
        {
          sc.baud = fromIndex((SerialSpeedIndex)_model.config.serial[i].baudIndex, SERIAL_SPEED_115200);
          if(serial)
          {
            serial->flush();
            serial->begin(sc);
          }
          _model.logger.info().log(F("UART")).log(i).log(sc.baud).log(msp ? F("msp") : F("")).logln(deb ? F("debug") : F(""));
        }
        else
        {
          if(serial)
          {
            serial->flush();
          }
          _model.logger.info().log(F("UART")).log(i).logln(F("free"));
        }

        /*Serial.print(i); Serial.print(' ');
        Serial.print(sc.baud); Serial.print(' ');
        Serial.print(sc.tx_pin); Serial.print(' ');
        Serial.print(sc.rx_pin); Serial.print(' ');
        Serial.print(sc.inverted); Serial.print(' ');
        Serial.print((uint32_t)serial, HEX); Serial.print(' ');
        Serial.println();*/
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
#if defined(ESP32)
      /*typedef HardwareSerial SerialDeviceType;
      SerialDeviceType serialWrapper0(0);
      SerialDeviceType serialWrapper1(1);
      SerialDeviceType serialWrapper2(2);
      static SerialDeviceAdapter<SerialDeviceType> uart0(serialWrapper0);
      static SerialDeviceAdapter<SerialDeviceType> uart1(serialWrapper1);
      static SerialDeviceAdapter<SerialDeviceType> uart2(serialWrapper2);
      */
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
        ppm.begin(model.config.pin[PIN_INPUT_RX], model.config.input.ppmMode);
        model.logger.info().log(F("PPM RX")).log(model.config.pin[PIN_INPUT_RX]).logln(model.config.input.ppmMode);
        return &ppm;
      }
      return NULL;
    }

  private:
    Model& _model;
};

}

#endif
