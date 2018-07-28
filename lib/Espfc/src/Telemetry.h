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
      _stream = (Stream*)Hardware::getSerialPort(_model.config.serial, SERIAL_FUNCTION_TELEMETRY_FRSKY);
      return 1;
    }

    int update()
    {
      if(!_stream) return 0;
      Stats::Measure measure(_model.state.stats, COUNTER_TELEMETRY);
      (*this)
        //<< _model.state.baroAltitude
        << F("")
      ;
      println();
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
