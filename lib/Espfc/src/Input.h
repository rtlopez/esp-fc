#ifndef _ESPFC_INPUT_H_
#define _ESPFC_INPUT_H_

#include "Model.h"
#include "Math.h"
#include "InputPPM.h"

namespace Espfc {

class Input
{
  public:
    Input(Model& model): _model(model) {}
    int begin()
    {
      ppm.begin(_model.config.ppmPin);
    }

    int update()
    {
      if(!ppm.hasNewData()) return 0;// avoid multiple reading channels

      for(size_t i = 0; i < INPUT_CHANNELS; ++i)
      {
        long us = ppm.getTime(_model.config.inputMap[i]);
        float v = Math::map3((float)us, _model.config.inputMin[i], _model.config.inputNeutral[i], _model.config.inputMax[i], -1, 0, 1);
        v = Math::bound(v, -1.f, 1.f);
        _model.state.inputUs[i] = us;
        _model.state.input[i] = v;
      }

      float fm = _model.state.input[_model.config.flightModeChannel];
      if(fm < -0.3) _model.state.flightMode = MODE_DISARMED;
      else if(fm > 0.3) _model.state.flightMode = MODE_RATE;
      else _model.state.flightMode = MODE_ANGLE;

      ppm.resetNewData();
      return 1;
    }

  private:
    Model& _model;

};

}

#endif
