#ifndef _ESPFC_SERIAL_H_
#define _ESPFC_SERIAL_H_

#include <Wire.h>
#include "Arduino.h"
#include "Model.h"

namespace Espfc {

class Hardware
{
  public:
    Hardware(Model& model): _model(model) {}

    int begin()
    {
      Wire.begin();
      //Wire.setClock(400000);
      Wire.setClock(1000000); // in real ~640kHz
      //Wire.setClockStretchLimit(100); // default 230
      _model.logger.info().log(F("I2C")).log(Wire.status()).logln(Wire.status() == 0 ? F("OK") : F("FAIL"));

      for(int i = SERIAL_UART_0; i < SERIAL_UART_COUNT; i++)
      {
        HardwareSerial * serial = getSerialPort((SerialPort)i);
        if(!serial) continue;

        bool blackbox = _model.config.serial[i].functionMask & SERIAL_FUNCTION_BLACKBOX;
        bool msp = _model.config.serial[i].functionMask & SERIAL_FUNCTION_MSP;
        bool deb = _model.config.serial[i].functionMask & SERIAL_FUNCTION_TELEMETRY_FRSKY;

        if(blackbox && msp)
        {
          int idx = _model.config.serial[i].blackboxBaudIndex == SERIAL_SPEED_INDEX_AUTO ? _model.config.serial[i].baudIndex : _model.config.serial[i].blackboxBaudIndex;
          int speed = fromIndex((SerialSpeedIndex)idx, SERIAL_SPEED_115200);
          serial->begin(speed);
          _model.logger.info().log(F("UART")).log(i).log(speed).log(F("msp")).logln(F("blackbox"));
        }
        else if(blackbox)
        {
          int idx = _model.config.serial[i].blackboxBaudIndex == SERIAL_SPEED_INDEX_AUTO ? _model.config.serial[i].baudIndex : _model.config.serial[i].blackboxBaudIndex;
          int speed = fromIndex((SerialSpeedIndex)idx, SERIAL_SPEED_250000);
          serial->begin(speed);
          _model.logger.info().log(F("UART")).log(i).log(speed).logln(F("blackbox"));
        }
        else if(msp || deb)
        {
          int speed = fromIndex((SerialSpeedIndex)_model.config.serial[i].baudIndex, SERIAL_SPEED_115200);
          serial->begin(speed);
          _model.logger.info().log(F("UART")).log(i).log(speed).log(msp ? F("msp") : F("")).logln(deb ? F("debug") : F(""));
        }
      }
      //Serial1.begin(115200);
      return 1;
    }

    static HardwareSerial * getSerialPort(SerialConfig * config, SerialFunction sf)
    {
      for (int i = SERIAL_UART_0; i < SERIAL_UART_COUNT; i++)
      {
        if(config[i].functionMask & sf) return getSerialPort((SerialPort)i);
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

    static HardwareSerial * getSerialPort(SerialPort portId)
    {
      switch(portId)
      {
        case SERIAL_UART_0:
          return  &Serial;
        case SERIAL_UART_1:
          return  &Serial1;
        default:
          return NULL;
      }
    }

  private:
    Model& _model;
};

}

#endif
