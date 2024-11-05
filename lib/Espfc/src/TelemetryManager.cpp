#include "TelemetryManager.h"

namespace Espfc {

TelemetryManager::TelemetryManager(Model& model): _model(model), _msp(model), _text(model), _crsf(model) {}

int TelemetryManager::process(Device::SerialDevice& s, TelemetryProtocol protocol) const
{
  Utils::Stats::Measure measure(_model.state.stats, COUNTER_TELEMETRY);

  switch(protocol)
  {
    case TELEMETRY_PROTOCOL_TEXT:
      _text.process(s);
      break;
    case TELEMETRY_PROTOCOL_CRSF:
      _crsf.process(s);
      break;
  }

  return 1;
}

int TelemetryManager::processMsp(Device::SerialDevice& s, TelemetryProtocol protocol, Connect::MspMessage m, uint8_t origin)
{
  Connect::MspResponse r;

  // not valid msp message, stop processing
  if(!m.isReady() || !m.isCmd()) return 0;

  _msp.processCommand(m, r, s);

  switch(protocol)
  {
    case TELEMETRY_PROTOCOL_CRSF:
      _crsf.sendMsp(s, r, origin);
      break;
    default:
      break;
  }

  _msp.postCommand();

  return 1;
}

}
