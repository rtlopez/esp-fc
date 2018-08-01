#ifndef _ESPFC_SERIAL_MANAGER_H_
#define _ESPFC_SERIAL_MANAGER_H_

#include <Arduino.h>
#include "Model.h"
#include "Hardware.h"
#include "Msp.h"
#include "Cli.h"
#include "Wireless.h"
#include "Telemetry.h"

namespace Espfc {

class SerialManager
{
  public:
    SerialManager(Model& model): _model(model), _msp(model), _cli(model), _wireless(model), _telemetry(model), _current(SERIAL_UART_0) {}

    int begin()
    {
      _wireless.begin();
      return 1;
    }

    int update()
    {
      SerialPortState& ss = _model.state.serial[_current];
      const SerialPortConfig& sc = _model.config.serial[_current];
      if(!ss.stream || sc.functionMask & SERIAL_FUNCTION_RX_SERIAL) {
        next();
        return 0;
      }

      Stream * stream = ss.stream;
      size_t count = 0;

      uint32_t now = millis();

      if(!ss.availableFrom && stream->available()) ss.availableFrom = now;
      bool timeout = ss.availableFrom && now - ss.availableFrom > 20;

      if(stream->available() > 3 || timeout)
      {
        ss.availableFrom = 0;
        while(stream->available())
        {
          char c = stream->read();
          if(sc.functionMask & SERIAL_FUNCTION_MSP)
          {
            bool consumed = _msp.process(c, ss.mspRequest, ss.mspResponse, *stream);
            if(!consumed)
            {
              _cli.process(c, ss.cliCmd, *stream);
            }
          }
          if(++count > 127) break;
        }
      }
      if(sc.functionMask & SERIAL_FUNCTION_TELEMETRY_FRSKY && _model.state.telemetryTimer.check())
      {
        _telemetry.process(*stream);
      }

      _wireless.update();

      next();

      return 1;
    }

  private:
    void next()
    {
      _current++;
      if(_current >= SERIAL_UART_COUNT) _current = 0;
    }

    Model& _model;
    Msp _msp;
    Cli _cli;
    Wireless _wireless;
    Telemetry _telemetry;
    size_t _current;
};

}

#endif