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
      EEPROM.begin(1024);

      Wire.begin();
      //Wire.setClock(400000);
      Wire.setClock(1000000); // in real ~640kHz

      if(_model.config.uart1Speed != SERIAL_SPEED_NONE)
      {
        Serial.begin(_model.config.uart1Speed);
      }
      if(_model.config.uart2Speed != SERIAL_SPEED_NONE)
      {
        Serial1.begin(_model.config.uart2Speed);
      }
    }

    int update()
    {

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
