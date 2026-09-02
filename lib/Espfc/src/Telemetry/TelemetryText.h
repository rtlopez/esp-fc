#pragma once

#include "Model.h"

namespace Espfc::Telemetry {

class TelemetryText
{
public:
  TelemetryText(Model& model): _model(model) {}

  int process(Stream::Printer& s) const
  {
    // print(s, _model.state.gyro.adc.x, 3);
    // println(s);

    return 1;
  }

private:
  template<typename T>
  void print(Stream::Printer& s, const T& v) const
  {
    s.print(v);
    s.print(' ');
  }

  void print(Stream::Printer& s, const long& v) const
  {
    s.print(v);
    s.print(' ');
  }

  void print(Stream::Printer& s, float& v, int len) const
  {
    s.print(v, len);
    s.print(' ');
  }

  void print(Stream::Printer& s, const VectorFloat& v) const
  {
    print(s, v.x);
    print(s, v.y);
    print(s, v.z);
  }

  void print(Stream::Printer& s, const VectorInt16& v) const
  {
    print(s, v.x);
    print(s, v.y);
    print(s, v.z);
  }

  void print(Stream::Printer& s, const Quaternion& v) const
  {
    print(s, v.w);
    print(s, v.x);
    print(s, v.y);
    print(s, v.z);
  }

  void println(Stream::Printer& s) const
  {
    s.println();
  }

  Model& _model;
};

} // namespace Espfc::Telemetry
