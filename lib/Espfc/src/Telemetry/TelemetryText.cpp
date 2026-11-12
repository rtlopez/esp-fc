#include "Telemetry/TelemetryText.hpp"

namespace Espfc::Telemetry {

int TelemetryText::process(Stream::Printer& s) const
{
  // print(s, _model.state.gyro.adc.x, 3);
  // println(s);

  return 1;
}

void TelemetryText::print(Stream::Printer& s, const long& v) const
{
  s.print(v);
  s.print(' ');
}

void TelemetryText::print(Stream::Printer& s, float& v, int len) const
{
  s.print(v, len);
  s.print(' ');
}

void TelemetryText::print(Stream::Printer& s, const VectorFloat& v) const
{
  print(s, v.x);
  print(s, v.y);
  print(s, v.z);
}

void TelemetryText::print(Stream::Printer& s, const VectorInt16& v) const
{
  print(s, v.x);
  print(s, v.y);
  print(s, v.z);
}

void TelemetryText::print(Stream::Printer& s, const Quaternion& v) const
{
  print(s, v.w);
  print(s, v.x);
  print(s, v.y);
  print(s, v.z);
}

void TelemetryText::println(Stream::Printer& s) const
{
  s.println();
}

} // namespace Espfc::Telemetry
