#ifndef _ESPFC_MIXER_H_
#define _ESPFC_MIXER_H_

#include "Model.h"
#include "EscDriver.h"

namespace Espfc {

class Mixer
{
  public:
    Mixer(Model& model): _model(model) {}

    int begin()
    {
      for(size_t i = 0; i < OUTPUT_CHANNELS; ++i)
      {
        ESCDriver.attach(i, _model.config.outputPin[i], _model.state.outputDisarmed[i]);
        _model.logger.info().log(F("OUTPUT PIN")).log(i).logln(_model.config.outputPin[i]);
      }
      ESCDriver.begin((EscProtocol)_model.config.outputProtocol, _model.config.outputAsync, _model.config.outputRate);
      _model.logger.info().log(F("OUTPUT CONF")).log(_model.config.outputProtocol).log(_model.config.outputAsync).logln(_model.config.outputRate);
      return 1;
    }

    int update()
    {
      _model.state.stats.start(COUNTER_MIXER);
      switch(_model.config.mixerType)
      {
        case FRAME_QUAD_X:
          updateQuadX();
          break;
        case FRAME_BALANCE_ROBOT:
          updateBalancingRobot();
          break;
      }
      _model.state.stats.end(COUNTER_MIXER);
      return 1;
    }

  private:
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
      float y = _model.state.output[AXIS_YAW] * (_model.config.yawReverse ? -1 : 1);
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
      if(_model.isActive(MODE_AIRMODE))
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

    void writeOutput(float * out, size_t axes)
    {
      bool stop = _stop();
      for(size_t i = 0; i < OUTPUT_CHANNELS; i++)
      {
        if(i >= axes || stop)
        {
          _model.state.outputUs[i] = _model.state.outputDisarmed[i];
        }
        else
        {
          float v = Math::bound(out[i], -1.f, 1.f);
          _model.state.outputUs[i] = (int16_t)Math::map3(v, -1.f, 0.f, 1.f, _model.config.outputMin[i], _model.config.outputNeutral[i], _model.config.outputMax[i]);
        }
      }
      _write(_model.state.outputUs, OUTPUT_CHANNELS);
    }

    void _write(int16_t * out, size_t axes)
    {
      for(size_t i = 0; i < OUTPUT_CHANNELS; i++)
      {
        ESCDriver.write(i, _model.state.outputUs[i]);
      }
      ESCDriver.apply();
    }

    bool _stop(void)
    {
      return !_model.isActive(MODE_ARMED) || (_model.isActive(FEATURE_MOTOR_STOP) && _model.state.inputUs[AXIS_THRUST] < _model.config.inputMinCheck);
    }

    Model& _model;
};

}

#endif
