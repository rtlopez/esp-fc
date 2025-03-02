#pragma once

#include "Model.h"
#include "Device/SerialDevice.h"
#include "Connect/MspProcessor.hpp"
#include "Connect/Vtx.hpp"
#include "Connect/Cli.hpp"
#include "TelemetryManager.h"
#include "Output/OutputIBUS.hpp"
#include "Sensor/GpsSensor.hpp"
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

private:
  static Device::SerialDevice * getSerialPortById(SerialPort portId);
  void processMsp(SerialPortState& ss);

  void next()
  {
    _current++;
    if(_current >= SERIAL_UART_COUNT) _current = 0;
  }

  Model& _model;
  size_t _current;

  Connect::MspProcessor _msp;
  Connect::Cli _cli;
  Connect::Vtx _vtx;
  TelemetryManager& _telemetry;
  Output::OutputIBUS _ibus;
  Sensor::GpsSensor _gps;
#ifdef ESPFC_SERIAL_SOFT_0_WIFI
  Wireless _wireless;
#endif
};

}
