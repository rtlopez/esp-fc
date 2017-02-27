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
#include "Telemetry.h"
#include "Cli.h"

namespace Espfc {

class Espfc
{
  public:
    Espfc(): _model(), _controller(_model), _input(_model), _actuator(_model), _sensor(_model), _fusion(_model), _mixer(_model), _telemetry(_model, Serial), _cli(_model, Serial) {}
    int begin()
    {
      _sensor.begin();
      _fusion.begin();
      _input.begin();
      _actuator.begin();
      _controller.begin();
      _mixer.begin();
      _telemetry.begin();
      _cli.begin();
      return 1;
    }

    int update()
    {
      bool updated = false;
      _sensor.update();
      _fusion.update();
      _input.update();
      _actuator.update();
      _controller.update();
      _mixer.update();
      _telemetry.update();
      _cli.update();
      _model.state.newGyroData = false;
      _model.state.newInputData = false;
      return 1;
    }
    Model& model() { return _model; }

  private:
    Model _model;
    Controller _controller;
    Input _input;
    Actuator _actuator;
    Sensor _sensor;
    Fusion _fusion;
    Mixer _mixer;
    Telemetry _telemetry;
    Cli _cli;
};

}

#endif
