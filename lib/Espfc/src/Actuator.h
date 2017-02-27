#ifndef _ESPFC_ACTUATOR_H_
#define _ESPFC_ACTUATOR_H_

#include "Model.h"
#include "Math.h"

namespace Espfc {

class Actuator
{
  public:
    Actuator(Model& model): _model(model) {}
    int begin()
    {
    }

    int update()
    {
      if(!_model.state.newInputData) return 0;
      updateFlightMode();
      updateScaler();
    }

  private:
    void updateScaler()
    {
      for(size_t i = 0; i < 3; i++)
      {
        short mode = _model.config.actuatorConfig[i];
        if(!mode) continue;
        short c = _model.config.actuatorChannels[i];
        float v = _model.state.input[c];
        float min = _model.config.actuatorMin[i];
        float max = _model.config.actuatorMax[i];
        float scale = Math::map3(v, -1.f, 0.f, 1.f, min, 1.f, max);
        for(size_t x = 0; x < 3; x++)
        {
          if(x == 0 && !(mode & ACT_AXIS_X)) continue;
          if(x == 1 && !(mode & ACT_AXIS_Y)) continue;
          if(x == 2 && !(mode & ACT_AXIS_Z)) continue;

          PidState& inner = _model.state.innerPid[i];
          if(mode & ACT_INNER_P) inner.pScale = scale;
          if(mode & ACT_INNER_I) inner.iScale = scale;
          if(mode & ACT_INNER_D) inner.dScale = scale;

          PidState& outer = _model.state.outerPid[i];
          if(mode & ACT_OUTER_P) inner.pScale = scale;
          if(mode & ACT_OUTER_I) inner.iScale = scale;
          if(mode & ACT_OUTER_D) inner.dScale = scale;
        }
      }
    }

    void updateFlightMode()
    {
      float v = _model.state.input[_model.config.flightModeChannel];
      int i = v > 0.3f ? 2 : (v > -0.3f ? 1 : 0);
      _model.state.flightMode = _model.config.flightModes[i];
    }
    Model& _model;
};

}

#endif
