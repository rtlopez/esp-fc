#pragma once

#include "Model.h"

namespace Espfc::Telemetry {

class TelemetryText
{
public:
  TelemetryText(Model& model): _model(model) {}

  int process(Stream::Printer& s) const;

private:
  template<typename T>
  void print(Stream::Printer& s, const T& v) const
  {
    s.print(v);
    s.print(' ');
  }

  void print(Stream::Printer& s, const long& v) const;
  void print(Stream::Printer& s, float& v, int len) const;
  void print(Stream::Printer& s, const VectorFloat& v) const;
  void print(Stream::Printer& s, const VectorInt16& v) const;
  void print(Stream::Printer& s, const Quaternion& v) const;
  void println(Stream::Printer& s) const;

  Model& _model;
};

} // namespace Espfc::Telemetry
