#ifndef _ESPFC_MIXER_H_
#define _ESPFC_MIXER_H_

#include "Model.h"
#include "OutputPWM.h"

namespace Espfc {

class Mixer
{
  public:
    Mixer(Model& model): _model(model) {}
    int begin()
    {
      for(size_t i = 0; i < OUTPUT_CHANNELS; ++i)
      {
        PWM.attach(i, _model.config.outputPin[i],  _model.config.outputMin[i]);
      }
      PWM.begin(_model.config.pwmRate);
    }

    int update()
    {
      unsigned long now = millis();
      if(_model.state.mixerTimestamp + _model.state.gyroSampleInterval > now) return 0;

      switch(_model.config.modelFrame)
      {
        case FRAME_DISARMED:
          updateDisarmed();
          break;
        case FRAME_QUAD_X:
          updateQuadX();
          break;
        case FRAME_BALANCE_ROBOT:
          updateBalancingRobot();
          break;
        default:
          break;
      }
      for(size_t i = 0; i < OUTPUT_CHANNELS; ++i)
      {
        PWM.write(i, _model.state.outputUs[i]);
      }
      _model.state.mixerTimestamp = now;

      return 1;
    }

    void updateDisarmed()
    {
      float out[4];
      out[0] = -1.f;
      out[1] = -1.f;
      out[2] = -1.f;
      out[3] = -1.f;

      writeOutput(out, 4);
    }

    void updateBalancingRobot()
    {
      float p = _model.state.output[AXIS_PITH];
      float t = _model.state.output[AXIS_THRUST];

      float out[2];
      out[0] = t + p;
      out[1] = t - p;

      writeOutput(out, 2);
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

      writeOutput(out, 4);
    }

    void writeOutput(float * out, int axes)
    {
      for(size_t i = 0; i < OUTPUT_CHANNELS; i++)
      {
        if(i >= axes)
        {
          _model.state.outputUs[i] = _model.config.outputMin[i];
        }
        else
        {
          float v = Math::bound(out[i], -1.f, 1.f);
          _model.state.outputUs[i] = (short)Math::map3(v, -1.f, 0.f, 1.f, _model.config.outputMin[i], _model.config.outputNeutral[i], _model.config.outputMax[i]);
        }
      }
    }
  private:
    Model& _model;
};

}

#endif
