#pragma once

namespace Espfc {

namespace Telemetry {

class TelemetryCRSF
{
public:
  TelemetryCRSF(Model& model): _model(model) {}

  int process(Device::SerialDevice& s) const
  {

    return 1;
  }

  int sendMsp(Device::SerialDevice& s, Msp::MspResponse r) const
  {

    return 1;
  }

private:
  Model& _model;
};

}

}
