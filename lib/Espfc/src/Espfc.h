#ifndef _ESPFC_ESPFC_H_
#define _ESPFC_ESPFC_H_

#include <Model.h>
#include <Controller.h>
#include <Input.h>
#include <Sensor.h>
#include <Fusion.h>
#include <Mixer.h>

namespace Espfc {

class Espfc
{
  public:
    Espfc(): _model(), _controller(_model), _input(_model), _sensor(_model), _fusion(_model), _mixer(_model) {}
    int begin()
    {
      _controller.begin();
      _input.begin();
      _sensor.begin();
      _mixer.begin();
    }

    int update()
    {
      _controller.update();
      _input.update();
      _sensor.update();
      _mixer.update();
    }

  private:
    Model _model;
    Controller _controller;
    Input _input;
    Sensor _sensor;
    Fusion _fusion;
    Mixer _mixer;
};

}

#endif
