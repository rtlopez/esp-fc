#pragma once

#include "Connect/Cli.hpp"
#include "Connect/MspProcessor.hpp"
#include "Connect/Vtx.hpp"
#include "Model.h"
#include "Output/OutputIBUS.hpp"
#include "Sensor/GpsSensor.hpp"
#include "Stream/ReadWritable.hpp"
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
  int reload(ModelChangeEvent event);
  int update();

private:
  static Stream::ReadWritable* getSerialPortById(SerialPort portId);
  void processMsp(SerialPortState& ss);

  void next()
  {
    _current++;
    if (_current >= SERIAL_UART_COUNT) _current = 0;
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

} // namespace Espfc
