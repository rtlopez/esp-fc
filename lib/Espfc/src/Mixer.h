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
      if(!_model.state.newGyroData && !_model.state.newInputData) return 0;
      if(!_model.state.armed)
      {
        updateDisarmed();
        return 0;
      }

      unsigned long now = millis();
      switch(_model.config.modelFrame)
      {
        case FRAME_QUAD_X:         updateQuadX(); break;
        case FRAME_BALANCE_ROBOT:  updateBalancingRobot(); break;
        case FRAME_GIMBAL:         updateGimbal(); break;
        case FRAME_UNCONFIGURED:
        default:
          updateDisarmed();
          break;
      }
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

    void updateGimbal()
    {
      float out[3];
      out[0] = -1.f;
      out[1] = -1.f;
      out[2] = -1.f;
      writeOutput(out, 3);
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
        PWM.write(i, _model.state.outputUs[i]);
      }
    }
  private:
    Model& _model;
};

}

#endif
