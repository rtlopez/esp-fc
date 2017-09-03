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
      PPM.begin(_model.config.ppmPin, _model.config.ppmMode);
      setFailSafe();
    }

    void setFailSafe()
    {
      for(size_t i = 0; i < INPUT_CHANNELS; ++i)
      {
        _model.state.inputUs[i] = 1500;
        _model.state.input[i] = 0.f;
      }
      _model.state.input[3] = -1.f; // throttle
      _model.state.inputUs[3] = 1000;
    }

    int update()
    {
      // avoid multiple reading channels
      if(!PPM.hasNewData())
      {
        // fail-safe
        _model.state.inputDelay = micros() - PPM.getStart();
        //if(diff > 100000) setFailSafe();
        return 0;
      }

      for(size_t i = 0; i < INPUT_CHANNELS; ++i)
      {
        long us = PPM.getTime(_model.config.inputMap[i]);
        _model.state.inputUs[i] = us;

        float v = Math::map3((float)us, _model.config.inputMin[i], _model.config.inputNeutral[i], _model.config.inputMax[i], -1.f, 0.f, 1.f);
        v = Math::bound(v, -1.f, 1.f);
        if(fabs(v) < _model.config.inputDeadband) v = 0.f;
        _model.state.input[i] = _model.state.input[i] * (1.f - _model.config.inputAlpha) + v * _model.config.inputAlpha;
      }

      _model.state.newInputData = true;

      PPM.resetNewData();
      return 1;
    }

  private:
    Model& _model;

};

}

#endif
