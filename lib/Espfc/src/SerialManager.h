#pragma once

#include "Model.h"
#include "Device/SerialDevice.h"
#include "Msp/MspProcessor.h"
#include "Cli.h"
#include "TelemetryManager.h"
#ifdef ESPFC_SERIAL_SOFT_0_WIFI
#include "Wireless.h"
#endif

namespace Espfc {

class SerialManager
{
  public:
    SerialManager(Model& model, TelemetryManager& telemetry);

    int begin();
    int update();

    static Device::SerialDevice * getSerialPortById(SerialPort portId);

  private:
    void next()
    {
      _current++;
      if(_current >= SERIAL_UART_COUNT) _current = 0;
    }

    Model& _model;
    Msp::MspProcessor _msp;
    Cli _cli;
#ifdef ESPFC_SERIAL_SOFT_0_WIFI
    Wireless _wireless;
#endif
    TelemetryManager& _telemetry;
    size_t _current;
};

}
