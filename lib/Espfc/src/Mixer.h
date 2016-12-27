#ifndef _ESPFC_MIXER_H_
#define _ESPFC_MIXER_H_

#include "Model.h"

namespace Espfc {

class Mixer
{
  public:
    Mixer(Model& model): _model(model) {}
    int begin() {}
    int update()
    {
      switch(_model.config.modelFrame)
      {
        case FRAME_QUAD_X:
          updateQuadX();
          break;
        default:
          break;
      }
      return 1;
    }

    void updateQuadX()
    {
      float r = _model.state.output[AXIS_ROLL];
      float p = _model.state.output[AXIS_PITH];
      float y = _model.state.output[AXIS_YAW];
      float t = _model.state.output[AXIS_THRUST];

      float out[4];
      out[0] = t + r + p + y;
      out[1] = t - r + p - y;
      out[2] = t - r - p + y;
      out[3] = t + r - p - y;

      for(size_t i = 0; i < 4; i++)
      {
        long val = (long)Math::map3(out[i], -1.f, 0.f, 1.f, _model.config.outputMin[i], _model.config.outputNeutral[i], _model.config.outputMax[i]);
        _model.state.outputUs[0] = Math::bound(val, _model.config.outputMin[i], _model.config.outputMax[i]);
      }
    }

  private:
    Model& _model;

};

}

#endif
