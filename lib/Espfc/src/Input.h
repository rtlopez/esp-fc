#ifndef _ESPFC_INPUT_H_
#define _ESPFC_INPUT_H_

#include "Model.h"
#include "Math.h"
#include "InputPPM.h"

namespace Espfc {

static const size_t INPUT_BUFF = 3;

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
      _model.state.inputUs[AXIS_THRUST] = 1000;
      _model.state.input[AXIS_THRUST] = -1.f;
    }

    int update()
    {
      static float step = 0;
      static uint32_t inputDt = 10000;
      static uint32_t prevTm = 0;

      if(PPM.hasNewData())
      {
        uint32_t now = micros();
        inputDt = Math::bound(now - prevTm, (uint32_t)1000, (uint32_t)30000);
        prevTm = now;
        _read();
        PPM.resetNewData();
        step = 0;
      }
      else
      {
        // fail-safe
        _model.state.inputDelay = micros() - PPM.getStart();
        if(_model.state.inputDelay > 500000)  // 0.5s
        {
          setFailSafe();
          return 0;
        }
      }

      float interpolationStep = (_model.state.loopDt * 1000000) / inputDt;
      if(step < 1.f) step += interpolationStep;

      for(size_t i = 0; i < INPUT_CHANNELS; ++i)
      {
        float curr = Math::map3((float)_get(i, 0), _model.config.inputMin[i], _model.config.inputNeutral[i], _model.config.inputMax[i], -1.f, 0.f, 1.f);
        float prev = Math::map3((float)_get(i, 1), _model.config.inputMin[i], _model.config.inputNeutral[i], _model.config.inputMax[i], -1.f, 0.f, 1.f);
        float val = i < 4 ? Math::bound(_interpolate(prev, curr, step), -1.f, 1.f) : curr;
        if(fabs(val) < _model.config.inputDeadband) val = 0.f;
        //_model.state.input[i] = _model.state.input[i] * (1.f - _model.config.inputAlpha) + val * _model.config.inputAlpha;
        _model.state.input[i] = val;
        _model.state.inputUs[i] = (uint16_t)lrintf(Math::map3(_model.state.input[i], -1.f, 0.f, 1.f, _model.config.inputMin[i], _model.config.inputNeutral[i], _model.config.inputMax[i]));
      }

      return 1;
    }

  private:
    float _interpolate(float left, float right, float step)
    {
      return (left * (1.f - step) + right * step);
    }

    void _read()
    {
      _shift();
      for(size_t i = 0; i < INPUT_CHANNELS; ++i)
      {
        _set(i, PPM.getTime(_model.config.inputMap[i]));
      }
    }

    void _shift()
    {
      for(size_t b = INPUT_BUFF - 1; b > 0; b--)
      {
        for(size_t i = 0; i < INPUT_CHANNELS; i++)
        {
          _buff[b][i] = _buff[b - 1][i];
        }
      }
    }

    uint32_t _get(size_t i, size_t b)
    {
      return _buff[b][i];
    }

    void _set(size_t i, uint32_t v)
    {
      for(size_t b = 1; b < INPUT_BUFF; b++)
      {
        v += _buff[b][i];
      }
      v /= INPUT_BUFF;
      _buff[0][i] = v;
    }

    Model& _model;
    uint16_t _buff[INPUT_BUFF][INPUT_CHANNELS];
};

}

#endif
