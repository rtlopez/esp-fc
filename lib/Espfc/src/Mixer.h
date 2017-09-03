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
      if(!_model.state.loopChanged) return 0;

      if(!_model.state.armed)
      {
        updateDisarmed();
        return 0;
      }

      switch(_model.config.modelFrame)
      {
        case FRAME_QUAD_X:
          updateQuadX();
          break;
        case FRAME_BALANCE_ROBOT:
          updateBalancingRobot();
          break;
        case FRAME_GIMBAL:
          updateGimbal();
          break;
        case FRAME_DIRECT:
          updateDirect();
          break;
        case FRAME_UNCONFIGURED:
        default:
          updateDisarmed();
          break;
      }
      return 1;
    }

  private:
    void updateDisarmed()
    {
      short out[4];
      out[0] = 1000;
      out[1] = 1000;
      out[2] = 1000;
      out[3] = 1000;
      writeOutputRaw(out, 4);
    }

    void updateGimbal()
    {
      float out[3];
      out[0] = 0.f;
      out[1] = 0.f;
      out[2] = 0.f;
      writeOutput(out, 3);
    }

    void updateDirect()
    {
      float r = _model.state.output[AXIS_ROLL];
      float p = _model.state.output[AXIS_PITCH];
      float y = _model.state.output[AXIS_YAW];
      float t = _model.state.output[AXIS_THRUST];
      float out[4];
      out[0] = r;
      out[1] = p;
      out[2] = y;
      out[3] = t;
      writeOutput(out, 4);
    }

    void updateBalancingRobot()
    {
      float p = _model.state.output[AXIS_PITCH];
      float y = _model.state.output[AXIS_YAW];
      float out[2];
      out[0] = p + y;
      out[1] = p - y;
      writeOutput(out, 2);
    }

    void updateQuadX()
    {
      float r = _model.state.output[AXIS_ROLL];
      float p = _model.state.output[AXIS_PITCH];
      float y = _model.state.output[AXIS_YAW];
      float t = _model.state.output[AXIS_THRUST];

      float out[4];
      out[0] = -r + p - y;
      out[1] = -r - p + y;
      out[2] =  r + p + y;
      out[3] =  r - p - y;

      /*
      float min = 0, max = 0, adj = 0;
      for(size_t i = 0; i < 4; i++)
      {
        if(out[i] > max) max = out[i];
        else if(out[i] < min) min = out[i];
      }
      if(min < -1.f) adj = min + 1.f;
      else if(max > 1.f) adj = max - 1.f;
      */

      for(size_t i = 0; i < 4; i++)
      {
        out[i] += t;
      }
      writeOutput(out, 4);
    }

    void writeOutput(float * out, int axes)
    {
      bool stop = _stop();
      for(size_t i = 0; i < OUTPUT_CHANNELS; i++)
      {
        if(i >= axes)
        {
          _model.state.outputUs[i] = _model.config.outputMin[i];
        }
        else
        {
          float v = Math::bound(out[i], -1.f, 1.f);
          _model.state.outputUs[i] = stop ? 1000 : (short)Math::map3(v, -1.f, 0.f, 1.f, _model.config.outputMin[i], _model.config.outputNeutral[i], _model.config.outputMax[i]);
        }
        PWM.write(i, _model.state.outputUs[i]);
      }
    }

    void writeOutputRaw(short * out, int axes)
    {
      bool stop = _stop();
      for(size_t i = 0; i < OUTPUT_CHANNELS; i++)
      {
        if(i >= axes)
        {
          _model.state.outputUs[i] = 1000;
        }
        else
        {
          _model.state.outputUs[i] = stop ? 1000 : Math::bound(out[i], (short)1000, (short)2000);
        }
        PWM.write(i, _model.state.outputUs[i]);
      }
    }

    bool _stop(void)
    {
      return (_model.config.lowThrottleMotorStop && _model.state.input[AXIS_THRUST] < _model.config.lowThrottleTreshold) || !_model.state.armed;
    }

    Model& _model;
};

}

#endif
