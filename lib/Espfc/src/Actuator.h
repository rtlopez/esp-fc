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
      _model.state.modeMask = 0;
    }

    int update()
    {
      _model.state.stats.start(COUNTER_ACTUATOR);
      updateModeMask();
      updateScaler();
      _model.state.stats.end(COUNTER_ACTUATOR);
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
        float scale = Math::map3(v, -1.f, 0.f, 1.f, min, min < 0 ? 0.f : 1.f, max);
        for(size_t x = 0; x < AXES; x++)
        {
          if(
            x == AXIS_ROLL   && (mode & ACT_AXIS_ROLL)  ||
            x == AXIS_PITCH  && (mode & ACT_AXIS_PITCH) ||
            x == AXIS_YAW    && (mode & ACT_AXIS_YAW)   ||
            x == AXIS_THRUST && (mode & ACT_AXIS_THRUST)
          )
          {

            if(mode & ACT_INNER_P) _model.state.innerPid[x].pScale = scale;
            if(mode & ACT_INNER_I) _model.state.innerPid[x].iScale = scale;
            if(mode & ACT_INNER_D) _model.state.innerPid[x].dScale = scale;

            if(mode & ACT_OUTER_P) _model.state.outerPid[x].pScale = scale;
            if(mode & ACT_OUTER_I) _model.state.outerPid[x].iScale = scale;
            if(mode & ACT_OUTER_D) _model.state.outerPid[x].dScale = scale;

          }
        }
      }
    }

    void updateModeMask()
    {
      uint32_t newMask;
      for(size_t i = 0; i < ACTUATOR_CONDITIONS; i++)
      {
        ActuatorCondition * c = &_model.config.conditions[i];
        if(!(c->min < c->max)) continue; // inactive

        uint16_t min = c->min * 25 + 900;
        uint16_t max = c->max * 25 + 900;
        size_t ch = c->ch + AXIS_AUX_1;
        if(ch < AXIS_AUX_1 || ch >= AXIS_COUNT) continue; // invalid channel

        uint16_t val = _model.state.inputUs[ch];
        if(val > min && val < max)
        {
          newMask |= 1 << c->id;
        }
      }
      _model.state.modeMask = newMask;
    }

    Model& _model;
};

}

#endif
