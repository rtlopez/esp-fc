#ifndef _ESPFC_TELEMETRY_H_
#define _ESPFC_TELEMETRY_H_

#include "Model.h"
#include "Hardware.h"

namespace Espfc {

class Telemetry
{
  public:
    Telemetry(Model& model): _model(model) {}

    int begin()
    {
      _stream = (Stream*)Hardware::getSerialPort((SerialPort)_model.config.telemetryPort);
      return 1;
    }

    int update()
    {
      if(!_stream) return 0;
      _model.state.stats.start(COUNTER_TELEMETRY);
      (*this)
        << ""
        //<< _model.state.timestamp
        //<< _model.state.armed
        //<< _model.state.gyroBiasValid

        //<< _model.state.accelRaw
        //<< _model.state.gyroRaw
        //<< _model.state.magRaw

        //<< _model.state.gyro
        //<< _model.state.accel
        //<< _model.state.mag
        //<< " | "
        //<< _model.state.rate
        //<< _model.state.angle

        //<< (int)degrees(_model.state.desiredAngle.x)
        //<< (int)degrees(_model.state.desiredAngle.y)
        //<< (int)degrees(_model.state.desiredAngle.z)

        //<< " | "

        //<< (int)degrees(_model.state.controlAngle.x)
        //<< (int)degrees(_model.state.controlAngle.y)
        //<< (int)degrees(_model.state.controlAngle.z)

        //<< " | "

        //<< (int)degrees(_model.state.desiredRate[0])
        //<< (int)degrees(_model.state.desiredRate[1])
        //<< (int)degrees(_model.state.desiredRate[2])

        //<< _model.state.desiredRotation
        //<< _model.state.desiredAngle

        //<< _model.state.outputUs[0]
        //<< _model.state.outputUs[1]
        //<< _model.state.outputUs[2]
        //<< _model.state.outputUs[3]
      ;
      println();
      _model.state.stats.end(COUNTER_TELEMETRY);
      return 1;
    }

  private:
    template<typename T>
    Telemetry& operator<<(T v)
    {
      if(!_stream) return *this;
      (*_stream).print(v);
      (*_stream).print(' ');
      return *this;
    }

    Telemetry& operator<<(long v)
    {
      if(!_stream) return *this;
      (*_stream).print(v);
      (*_stream).print(' ');
      return *this;
    }

    Telemetry& operator<<(float v)
    {
      if(!_stream) return *this;
      (*_stream).print(v, 4);
      (*_stream).print(' ');
      return *this;
    }

    Telemetry& operator<<(const VectorFloat& v)
    {
      (*this) << v.x << v.y << v.z;
      return *this;
    }

    Telemetry& operator<<(const VectorInt16& v)
    {
      (*this) << v.x << v.y << v.z;
      return *this;
    }

    Telemetry& operator<<(const Quaternion& q)
    {
      (*this) << q.w << q.x << q.y << q.z;
      return *this;
    }

    void println()
    {
      if(!_stream) return;
      (*_stream).println();
    }

    Model& _model;
    Stream * _stream;
};

}

#endif
