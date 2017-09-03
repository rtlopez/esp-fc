#ifndef _ESPFC_ESPFC_H_
#define _ESPFC_ESPFC_H_

#include <Wire.h>

#include "Model.h"
#include "Controller.h"
#include "Input.h"
#include "Actuator.h"
#include "Sensor.h"
#include "Fusion.h"
#include "Mixer.h"
#include "Blackbox.h"
#include "Telemetry.h"
#include "Cli.h"
#include "Hardware.h"

namespace Espfc {

class Espfc
{
  public:
    Espfc():
      _model(), _hardware(_model), _controller(_model), _input(_model), _actuator(_model), _sensor(_model), _fusion(_model),
      _mixer(_model), _blackbox(_model), _telemetry(_model), _cli(_model)
      {}

    int begin()
    {
      _hardware.begin();
      _sensor.begin();
      _fusion.begin();
      _input.begin();
      _actuator.begin();
      _controller.begin();
      _mixer.begin();
      _blackbox.begin();
      _telemetry.begin();
      _cli.begin();

      return 1;
    }

    int update()
    {
      _hardware.update();
      _sensor.update();
      _fusion.update();
      _input.update();
      _actuator.update();
      _controller.update();
      _mixer.update();
      _blackbox.update();
      _telemetry.update();
      _cli.update();

      _model.state.gyroChanged = false;
      _model.state.loopChanged = false;

      return 1;
    }

  private:
    Model _model;
    Hardware _hardware;
    Controller _controller;
    Input _input;
    Actuator _actuator;
    Sensor _sensor;
    Fusion _fusion;
    Mixer _mixer;
    Blackbox _blackbox;
    Telemetry _telemetry;
    Cli _cli;
};

}

#endif
