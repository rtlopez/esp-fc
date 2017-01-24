#ifndef _ESPFC_ESPFC_H_
#define _ESPFC_ESPFC_H_

#include <Wire.h>

#include "Model.h"
#include "Controller.h"
#include "Input.h"
#include "Sensor.h"
#include "Fusion.h"
#include "Mixer.h"
#include "Telemetry.h"

namespace Espfc {

class Espfc
{
  public:
    Espfc(): _model(), _controller(_model), _input(_model), _sensor(_model), _fusion(_model), _mixer(_model), _telemetry(_model) {}
    int begin()
    {
      _sensor.begin();
      _fusion.begin();
      _input.begin();
      _controller.begin();
      _mixer.begin();
      _telemetry.begin();
      return 1;
    }

    int update()
    {
      while(_sensor.update())
      {
        _fusion.update();
      }
      _input.update();
      _controller.update();
      _mixer.update();
      _telemetry.update();
      return 1;
    }
    Model& model() { return _model; }

  private:
    Model _model;
    Controller _controller;
    Input _input;
    Sensor _sensor;
    Fusion _fusion;
    Mixer _mixer;
    Telemetry _telemetry;
};

}

#endif
