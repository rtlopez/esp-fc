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

namespace Espfc {

class Espfc
{
  public:
    Espfc(HardwareSerial& cliPort, HardwareSerial& telPort):
      _model(), _controller(_model), _input(_model), _actuator(_model), _sensor(_model), _fusion(_model),
      _mixer(_model), _blackbox(_model, cliPort), _telemetry(_model, cliPort), _cli(_model, cliPort)
      {}
        
    int begin()
    {
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
      bool updated = false;
      _sensor.update();
      _fusion.update();
      _input.update();
      _actuator.update();
      _controller.update();
      _mixer.update();
      _blackbox.update();
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
    Blackbox _blackbox;
    Telemetry _telemetry;
    Cli _cli;
};

}

#endif
