#ifndef _ESPFC_MIXER_H_
#define _ESPFC_MIXER_H_

#include "Model.h"
#include "OutputPWMFast.h"
#include "OutputPWM.h"

namespace Espfc {

//#define PWMDriver PWM
#define PWMDriver PWMfast

class Mixer
{
  public:
    Mixer(Model& model): _model(model) {}
    int begin()
    {
      for(size_t i = 0; i < OUTPUT_CHANNELS; ++i)
      {
        PWMDriver.attach(i, _model.config.outputPin[i],  1000);
      }
      PWMDriver.begin(_model.config.outputRate, (OutputProtocol)_model.config.outputProtocol);
    }

    int update()
    {
      if(!_model.state.armed)
      {
        _model.state.stats.start(COUNTER_MIXER);
        updateDisarmed();
        _model.state.stats.end(COUNTER_MIXER);
        return 0;
      }

      _model.state.stats.start(COUNTER_MIXER);
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
      _model.state.stats.end(COUNTER_MIXER);
      return 1;
    }

  private:
    void updateDisarmed()
    {
      writeOutputRaw(_model.state.outputDisarmed, OUTPUT_CHANNELS);
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
      float r = -1.f + abs(_model.state.output[AXIS_ROLL]);
      float p = -1.f + abs(_model.state.output[AXIS_PITCH]);
      float y = -1.f + abs(_model.state.output[AXIS_YAW]);
      float t = -1.f + abs(_model.state.output[AXIS_THRUST]);
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

      float out[OUTPUT_CHANNELS];
      for(size_t i = 0; i < OUTPUT_CHANNELS; i++)
      {
        out[i] = 0;
      }

      // TODO: use mix table
      out[0] = -r + p + y;
      out[1] = -r - p - y;
      out[2] =  r + p - y;
      out[3] =  r - p + y;

      // airmode logic
      if(false)
      {
        float min = 0, max = 0;
        for(size_t i = 0; i < OUTPUT_CHANNELS; i++)
        {
          max = std::max(max, out[i]);
          min = std::min(min, out[i]);
        }
        float range = (max - min) / 2.f;
        if(range > 1.f)
        {
          for(size_t i = 0; i < OUTPUT_CHANNELS; i++)
          {
            out[i] /= range;
          }
          t = Math::bound(t, -1.f + range, 1.f - range);
        }
      }

      for(size_t i = 0; i < OUTPUT_CHANNELS; i++)
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
          _model.state.outputUs[i] = _model.state.outputDisarmed[i];
        }
        else
        {
          float v = Math::bound(out[i], -1.f, 1.f);
          _model.state.outputUs[i] = stop ? _model.state.outputDisarmed[i] : (short)Math::map3(v, -1.f, 0.f, 1.f, _model.config.outputMin[i], _model.config.outputNeutral[i], _model.config.outputMax[i]);
        }
        PWMDriver.write(i, _model.state.outputUs[i]);
      }
      PWMDriver.apply();
    }

    void writeOutputRaw(uint16_t * out, int axes)
    {
      bool stop = _stop();
      for(size_t i = 0; i < OUTPUT_CHANNELS; i++)
      {
        if(i >= axes)
        {
          _model.state.outputUs[i] = _model.state.outputDisarmed[i];
        }
        else
        {
          _model.state.outputUs[i] = stop ? _model.state.outputDisarmed[i] : Math::bound(out[i], (uint16_t)1000, (uint16_t)2000);
        }
        PWMDriver.write(i, _model.state.outputUs[i]);
      }
      PWMDriver.apply();
    }

    bool _stop(void)
    {
      return (_model.config.lowThrottleMotorStop && _model.state.inputUs[AXIS_THRUST] < _model.config.lowThrottleTreshold) || !_model.state.armed;
    }

    Model& _model;
};

}

#endif
