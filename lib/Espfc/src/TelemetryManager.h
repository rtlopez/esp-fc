#pragma once

#include "Model.h"
#include "Device/SerialDevice.h"
#include "Telemetry/TelemetryText.h"
#include "Telemetry/TelemetryCRSF.h"
#include "Connect/Msp.hpp"
#include "Connect/MspProcessor.hpp"

namespace Espfc {

enum TelemetryProtocol {
  TELEMETRY_PROTOCOL_TEXT,
  TELEMETRY_PROTOCOL_CRSF,
};

class TelemetryManager
{
public:
  TelemetryManager(Model& model);
  int process(Device::SerialDevice& s, TelemetryProtocol protocol) const;
  int processMsp(Device::SerialDevice& s, TelemetryProtocol protocol, Connect::MspMessage m, uint8_t origin);

private:
  Model& _model;
  Connect::MspProcessor _msp;
  Telemetry::TelemetryText _text;
  Telemetry::TelemetryCRSF _crsf;
};

}
