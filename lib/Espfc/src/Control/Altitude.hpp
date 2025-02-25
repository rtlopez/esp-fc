#pragma once

#include "Model.h"
#include "Utils/Filter.h"

namespace Espfc::Control {

class Altitude
{
public:
  Altitude(Model& model): _model(model) {}

  int begin()
  {
    _model.state.altitude.height = 0.0f;
    _model.state.altitude.vario = 0.0f;

    _altitudeFilter.begin(FilterConfig(FILTER_PT2, 5), _model.state.accel.timer.rate);
    _varioFilter.begin(FilterConfig(FILTER_PT2, 5), _model.state.accel.timer.rate);

    return 1;
  }

  int update()
  {
    _model.state.altitude.height = _altitudeFilter.update(_model.state.baro.altitudeGround);

    float baroVario = _varioFilter.update(_model.state.baro.vario);
    _model.state.altitude.vario = baroVario;

    if(_model.config.debug.mode == DEBUG_ALTITUDE)
    {
      _model.state.debug[0] = std::clamp(lrintf(_model.state.altitude.height * 100.0f), -32000l, 32000l);   // gps trust
      _model.state.debug[1] = std::clamp(lrintf(_model.state.altitude.vario * 100.0f), -32000l, 32000l);    // baroAlt cm
      //_model.state.debug[1] = std::clamp(lrintf(accVario * 100.0f), -3200l, 32000l);                        // gpsAlt cm
      _model.state.debug[1] = std::clamp(lrintf(baroVario * 100.0f), -3200l, 32000l);                       // vario
    }

    return 1;
  }

private:
  Model& _model;
  Utils::Filter _altitudeFilter;
  Utils::Filter _varioFilter;
};

}
