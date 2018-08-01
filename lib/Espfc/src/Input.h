#ifndef _ESPFC_INPUT_H_
#define _ESPFC_INPUT_H_

#include "Model.h"
#include "Math.h"
#include "Hardware.h"
#include "InputPPM.h"
#include "InputSBUS.h"

namespace Espfc {

enum FailsafeChannelMode {
  FAILSAFE_MODE_AUTO,
  FAILSAFE_MODE_HOLD,
  FAILSAFE_MODE_SET
};

class Input
{
  public:
    Input(Model& model): _model(model) {}
    int begin()
    {
      _device = Hardware::getInputDevice(_model);
      setFailsafe();
      return 1;
    }

    void setFailsafe()
    {
      for(size_t i = 0; i < INPUT_CHANNELS; ++i)
      {
        const InputChannelConfig& ich = _model.config.input.channel[i];
        if(ich.fsMode == FAILSAFE_MODE_HOLD) continue;
        _model.state.inputUs[i] = ich.fsValue;
        _model.state.input[i] = Math::map((float)ich.fsValue, 1000.f, 2000.f, -1.f, 1.f);
      }
    }

    int update()
    {
      if(!_device) return 0;

      static float step = 0;
      static float inputDt = 0.02f;
      static uint32_t prevTm = 0;

      {
        Stats::Measure readMeasure(_model.state.stats, COUNTER_INPUT_READ);
        InputStatus status = _device->update();

        if(status == INPUT_FAILED)
        {
          setFailsafe();
          _model.state.buzzer.play(BEEPER_RX_LOST);
          _model.state.inputLinkValid = false;
          return 0;
        }

        if(status == INPUT_RECEIVED)
        {
          _model.state.inputLinkValid = true;
          _read();
          step = 0.f;
          switch(_model.config.input.interpolationMode)
          {
            case INPUT_INTERPOLATION_AUTO:
              {
                uint32_t now = micros();
                inputDt = Math::bound(now - prevTm, (uint32_t)4000, (uint32_t)40000) * 0.000001f;
                prevTm = now;
              }
              break;
            case INPUT_INTERPOLATION_MANUAL:
              inputDt = _model.config.input.interpolationInterval * 0.001f;
              break;
            case INPUT_INTERPOLATION_OFF:
              for(size_t i = 0; i < INPUT_CHANNELS; ++i)
              {
                _model.state.inputUs[i] = (float)_get(i, 0);
              }
              break;
            default:
              inputDt = 0.02f;
              break;
          }
        }
      }

      {
        Stats::Measure filterMeasure(_model.state.stats, COUNTER_INPUT_FILTER);
        if(_model.config.input.interpolationMode != INPUT_INTERPOLATION_OFF)
        {
          const float loopDt = _model.state.loopTimer.getDelta();
          const float interpolationStep = loopDt / inputDt;
          if(step < 1.f)
          {
            step += interpolationStep;
          }
          for(size_t i = 0; i < INPUT_CHANNELS; ++i)
          {
            float val = (float)_get(i, 0);
            if(i < INTERPOLETE_COUNT)
            {
              float prev = (float)_get(i, 1);
              val =_interpolate(prev, val, step);
            }
            _model.state.inputUs[i] = val;
          }
          if(_model.config.debugMode == DEBUG_RC_INTERPOLATION)
          {
            _model.state.debug[0] = 1000 * inputDt;
            _model.state.debug[1] = 10000 * loopDt;
            _model.state.debug[2] = 1000 * interpolationStep;
            _model.state.debug[3] = 1000 * step;
          }
        }

        for(size_t i = 0; i < INPUT_CHANNELS; ++i)
        {
          const InputChannelConfig& ich = _model.config.input.channel[i];
          _model.state.input[i] = Math::map(_model.state.inputUs[i], ich.min, ich.max, -1.f, 1.f);
        }
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
      for(size_t c = 0; c < INPUT_CHANNELS; ++c)
      {
        int pulse = _device->get(_model.config.input.channel[c].map);
        _set(c, pulse);
      }
    }

    void _shift()
    {
      for(size_t b = INPUT_BUFF_SIZE - 1; b > 0; b--)
      {
        for(size_t c = 0; c < INPUT_CHANNELS; c++)
        {
          _buff[b][c] = _buff[b - 1][c];
        }
      }
    }

    int16_t _get(size_t c, size_t b)
    {
      int16_t v = (_buff[b][c] + _buff[b + 1][c] + 1) >> 1; // avg last two samples
      //int v = _buff[b][c];
      return v;
    }

    void _set(size_t c, int16_t v)
    {
      const InputChannelConfig& ich = _model.config.input.channel[c];
      v -= _model.config.input.midRc - 1500;
      float t = Math::map3((float)v, (float)ich.min, (float)ich.neutral, (float)ich.max, 1000.f, 1500.f, 2000.f);
      t = Math::bound(t, 800.f, 2200.f);
      _buff[0][c] = _deadband(c, lrintf(t));
    }

    int16_t _deadband(size_t c, int16_t v)
    {
      if(c >= 3) return v;
      return Math::deadband(v - 1500, (int)_model.config.input.deadband) + 1500;
    }

    static const size_t INPUT_BUFF_SIZE = 3;

    Model& _model;
    int16_t _buff[INPUT_BUFF_SIZE][INPUT_CHANNELS];
    InputDevice * _device;
    static const size_t INTERPOLETE_COUNT = 4;
};

}

#endif
