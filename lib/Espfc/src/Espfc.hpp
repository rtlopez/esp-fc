#pragma once

#include "Blackbox/Blackbox.hpp"
#include "Connect/Buzzer.hpp"
#include "Control/Actuator.hpp"
#include "Control/Controller.hpp"
#include "Hardware.hpp"
#include "Input.hpp"
#include "Model.hpp"
#include "Output/Mixer.hpp"
#include "SensorManager.hpp"
#include "SerialManager.hpp"
#include "TelemetryManager.hpp"

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

  bool beginGyroTimer(Hal::HwTimer::Callback callback)
  {
    return _model.state.gyro.hwTimer.begin(getGyroInterval(), callback);
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
