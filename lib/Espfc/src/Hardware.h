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
      _model.logger.info().log(F("I2C")).logln(Wire.status());

      if(_model.config.uart1Speed != SERIAL_SPEED_NONE)
      {
        Serial.begin(_model.config.uart1Speed);
        _model.logger.info().log(F("UART0")).logln(_model.config.uart1Speed);
      }
      if(_model.config.uart2Speed != SERIAL_SPEED_NONE)
      {
        Serial1.begin(_model.config.uart2Speed);
        _model.logger.info().log(F("UART1")).logln(_model.config.uart2Speed);
      }
      return 1;
    }

    int update()
    {
      return 0;
    }

    static HardwareSerial * getSerialPort(SerialPort portId)
    {
      switch(portId)
      {
        case SERIAL_UART_1:
          return  &Serial;
        case SERIAL_UART_2:
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
