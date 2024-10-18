#pragma once

#include "Model.h"
#include "Hardware.h"
#include "Controller.h"
#include "Input.h"
#include "Actuator.h"
#include "SensorManager.h"
#include "SerialManager.h"
#include "Output/Mixer.h"
#include "Blackbox/Blackbox.h"
#include "Buzzer.h"

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
      return _model.state.gyroTimer.interval;
    }

  private:
    Model _model;
    Hardware _hardware;
    Controller _controller;
    Input _input;
    Actuator _actuator;
    SensorManager _sensor;
    Output::Mixer _mixer;
    Blackbox::Blackbox _blackbox;
    Buzzer _buzzer;
    SerialManager _serial;
    uint32_t _loop_next;
};

}
