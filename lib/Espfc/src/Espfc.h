#pragma once

#include "Blackbox/Blackbox.h"
#include "Connect/Buzzer.hpp"
#include "Control/Actuator.h"
#include "Control/Controller.h"
#include "Hardware.h"
#include "Input.h"
#include "Model.h"
#include "Output/Mixer.h"
#include "SensorManager.h"
#include "SerialManager.h"
#include "TelemetryManager.h"

namespace Espfc {

class Espfc
{
public:
  Espfc();

  int load();
  int begin();
  int update(bool externalTrigger = false);
  int updateOther();

  int getGyroInterval() const
  {
    return _model.state.gyro.timer.interval;
  }

private:
  Model _model;
  Hardware _hardware;
  Control::Controller _controller;
  TelemetryManager _telemetry;
  Input _input;
  Control::Actuator _actuator;
  SensorManager _sensor;
  Output::Mixer _mixer;
  Blackbox::Blackbox _blackbox;
  Connect::Buzzer _buzzer;
  SerialManager _serial;
  uint32_t _loop_next;
};

} // namespace Espfc
