#pragma once

#include "Connect/Msp.hpp"
#include "Connect/MspProcessor.hpp"
#include "Model.h"
#include "Stream/ReadWritable.hpp"
#include "Telemetry/TelemetryCRSF.hpp"
#include "Telemetry/TelemetryText.hpp"

namespace Espfc {

enum TelemetryProtocol
{
  TELEMETRY_PROTOCOL_TEXT,
  TELEMETRY_PROTOCOL_CRSF,
};

class TelemetryManager
{
public:
  TelemetryManager(Model& model);
  int process(Stream::ReadWritable& s, TelemetryProtocol protocol) const;
  int processMsp(Stream::ReadWritable& s, TelemetryProtocol protocol, Connect::MspMessage m, uint8_t origin);

private:
  Model& _model;
  Connect::MspProcessor _msp;
  Telemetry::TelemetryText _text;
  Telemetry::TelemetryCRSF _crsf;
};

} // namespace Espfc
