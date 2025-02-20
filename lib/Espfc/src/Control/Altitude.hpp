#pragma once

#include "Model.h"

namespace Espfc::Control {

class Altitude
{
public:
  Altitude(Model& model): _model(model) {}

  int begin()
  {
    _model.state.altitude.height = 0.0f;
    _model.state.altitude.heightPrev = 0.0f;
    _model.state.altitude.rate = 0.0f;

    return 1;
  }

  int update()
  {
    // initialy use baro altitude only, in feature mix with other sources according to trust level
    _model.state.altitude.height = _model.state.altitude.height + _model.state.baro.altitude;
    _model.state.altitude.rate = (_model.state.altitude.height - _model.state.altitude.heightPrev) * _model.state.loopTimer.rate;
    _model.state.altitude.heightPrev = _model.state.altitude.height;

    return 1;
  }

private:
  Model& _model;
};

}
