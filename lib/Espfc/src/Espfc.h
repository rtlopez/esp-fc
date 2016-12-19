#ifndef _ESPFC_ESPFC_H_
#define _ESPFC_ESPFC_H_

#include <Model.h>
#include <Controller.h>
#include <Input.h>
#include <Sensor.h>
#include <Mixer.h>

namespace Espfc {

class Espfc
{
  public:
    Espfc() {}
    int begin()
    {
      _model.begin();
      _controller.begin();
      _input.begin();
      _sensor.begin();
      _mixer.begin();
    }

    int update()
    {
      _model.update();
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
    Mixer _mixer;
};

}

#endif
