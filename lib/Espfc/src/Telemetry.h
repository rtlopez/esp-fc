#ifndef _ESPFC_TELEMETRY_H_
#define _ESPFC_TELEMETRY_H_

#include "Model.h"

namespace Espfc {

class Telemetry
{
  public:
    Telemetry(Model& model): _model(model) {}
    int begin()
    {

    }

    int update()
    {
      int32_t now = millis();
      if(_model.config.telemetry && _model.config.telemetryInterval > 20 && now - _model.config.telemetryTimestamp > _model.config.telemetryInterval)
      {
        (*this)
          //<< _model.state.timestamp
          //<< _model.state.accelRaw.x
          //<< _model.state.accelRaw.y
          //<< _model.state.accelRaw.z
          //<< _model.state.gyroRaw.x
          //<< _model.state.gyroRaw.y
          //<< _model.state.gyroRaw.z
          << _model.state.accel.x
          << _model.state.accel.y
          << _model.state.accel.z
          << _model.state.gyro.x
          << _model.state.gyro.y
          << _model.state.gyro.z
        ;
        println();
        _model.config.telemetryTimestamp = now;
        return 1;
      }
      return 0;
    }

  private:
    template<typename T>
    Telemetry& operator<<(T v)
    {
      Serial.print(v); Serial.print(' ');
      return *this;
    }

    Telemetry& operator<<(float v)
    {
      Serial.print(v, 4); Serial.print(' ');
      return *this;
    }

    void println()
    {
      Serial.println();
    }

    Model& _model;
};

}

#endif
