#pragma once

#include "Model.hpp"
#include "Rc/Crsf.hpp"
#include "Stream/ReadWritable.hpp"

namespace Espfc {

namespace Telemetry {

enum TelemetryState
{
  CRSF_TELEMETRY_STATE_ATTI,
  CRSF_TELEMETRY_STATE_BAT,
  CRSF_TELEMETRY_STATE_FM,
  CRSF_TELEMETRY_STATE_GPS,
  CRSF_TELEMETRY_STATE_BARO,
  CRSF_TELEMETRY_STATE_HB,
};

class TelemetryCRSF
{
public:
  TelemetryCRSF(Model& model): _model(model) {}

  int begin();
  int process(Stream::ReadWritable& s) const;
  int sendMsp(Stream::ReadWritable& s, Connect::MspResponse resp, uint8_t origin);
  void send(const Rc::CrsfMessage& msg, Stream::ReadWritable& s) const;
  int16_t toAngle(float angle) const;
  void attitude(Rc::CrsfMessage& msg) const;
  void battery(Rc::CrsfMessage& msg) const;
  void flightMode(Rc::CrsfMessage& msg) const;
  void gps(Rc::CrsfMessage& msg) const;
  void vario(Rc::CrsfMessage& msg) const;
  void heartbeat(Rc::CrsfMessage& msg) const;

private:
  Model& _model;
  mutable TelemetryState _current;
  uint8_t _buff[256] = {0};
  uint8_t _seq = 0;
};

} // namespace Telemetry

} // namespace Espfc
