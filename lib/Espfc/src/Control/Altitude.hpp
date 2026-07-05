#pragma once

#include "Model.h"
#include "Utils/Filter.h"
#include <Complementary.h>

namespace Espfc::Control {

class Altitude
{
public:
  Altitude(Model& model): _model(model) {}

  int begin()
  {
    _model.state.altitude.height = 0.0f;
    _model.state.altitude.vario = 0.0f;

    _altitudeFilter.begin(FilterConfig(FILTER_PT3, 5), _model.state.accel.timer.rate);
    _varioFilter.begin(FilterConfig(FILTER_PT3, 5), _model.state.accel.timer.rate);
    _varioFusion.begin(_model.state.accel.timer.rate, _model.config.altHold.baroTau * 0.1f);

    return 1;
  }

  int update()
  {
    // upsampling filter to match imu rate
    auto baroAlt = _altitudeFilter.update(_model.state.baro.altitudeGround);
    auto baroVario = _varioFilter.update(_model.state.baro.vario);
    
    // complementary filter to fuse baro and accel
    auto accZ = _model.state.accel.world.z;
    _model.state.altitude.vario = _varioFusion.update(accZ, baroVario);
    _model.state.altitude.height = baroAlt;

    if(_model.config.debug.mode == DEBUG_ALTITUDE)
    {
      _model.state.debug[0] = std::clamp(lrintf(_model.state.baro.altitudeGround * 100.0f), -32000l, 32000l);  // gps trust
      _model.state.debug[1] = std::clamp(lrintf(_model.state.baro.vario * 100.0f), -32000l, 32000l);           // baroAlt cm
      _model.state.debug[2] = std::clamp(lrintf(_model.state.altitude.height * 100.0f), -32000l, 32000l);      // gpsAlt cm
      _model.state.debug[3] = std::clamp(lrintf(_model.state.altitude.vario * 100.0f), -32000l, 32000l);       // vario
    }

    return 1;
  }

private:
  Model& _model;
  Utils::Filter _altitudeFilter;
  Utils::Filter _varioFilter;
  Complementary _varioFusion;
};

}
