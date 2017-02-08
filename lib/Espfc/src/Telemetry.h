#ifndef _ESPFC_TELEMETRY_H_
#define _ESPFC_TELEMETRY_H_

#include "Model.h"

namespace Espfc {

class Telemetry
{
  public:
    Telemetry(Model& model, Stream& stream): _model(model), _stream(stream) {}
    int begin()
    {

    }

    int update()
    {
      unsigned long now = millis();
      if(_model.config.telemetry && _model.config.telemetryInterval >= 10 && _model.state.telemetryTimestamp + _model.config.telemetryInterval < now)
      {
        (*this)
          //<< _model.state.timestamp

          //<< _model.state.accelRaw.x << _model.state.accelRaw.y << _model.state.accelRaw.z
          //<< _model.state.gyroRaw.x  << _model.state.gyroRaw.y  << _model.state.gyroRaw.z
          //<< _model.state.magRaw.x   << _model.state.magRaw.y   << _model.state.magRaw.z
          //<< _model.state.gyroPose
          //<< _model.state.accelPose
          << _model.state.pose
          << _model.state.angle

          //<< _model.state.flightMode
          //<< _model.state.rateDesired[0]
          //<< _model.state.rateDesired[1]

          //<< _model.state.outputUs[0]
          //<< _model.state.outputUs[1]
          //<< _model.state.outputUs[2]
          //<< _model.state.outputUs[3]

          //<< _model.state.input[0]
          //<< _model.state.input[1]
          //<< _model.state.input[2]
          //<< _model.state.input[3]
          //<< _model.state.input[4]
          //<< _model.state.input[5]
          //<< _model.state.input[6]
          //<< _model.state.input[7]
        ;
        println();
        _model.state.telemetryTimestamp = now;
        return 1;
      }
      return 0;
    }

  private:
    template<typename T>
    Telemetry& operator<<(T v)
    {
      _stream.print(v); _stream.print(' ');
      return *this;
    }

    Telemetry& operator<<(float v)
    {
      _stream.print(v, 4); _stream.print(' ');
      return *this;
    }

    Telemetry& operator<<(const VectorFloat& v)
    {
      (*this) << v.x << v.y << v.z;
      return *this;
    }

    Telemetry& operator<<(const Quaternion& v)
    {
      (*this) << v.w << v.x << v.y << v.z;
      return *this;
    }

    void println()
    {
      _stream.println();
    }

    Model& _model;
    Stream& _stream;
};

}

#endif
