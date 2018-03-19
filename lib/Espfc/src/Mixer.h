#ifndef _ESPFC_MIXER_H_
#define _ESPFC_MIXER_H_

#include "Model.h"
#include "Hardware.h"
#include "EscDriver.h"

namespace Espfc {

class Mixer
{
  public:
    Mixer(Model& model): _model(model), _driver(NULL) {}

    int begin()
    {
      _driver = Hardware::getEscDriver(_model);
      if(!_driver) return 0;

      _model.state.minThrottle = _model.config.output.minThrottle;
      _model.state.maxThrottle = _model.config.output.maxThrottle;
      _model.state.digitalOutput = _model.config.output.protocol >= ESC_PROTOCOL_DSHOT150;
      if(_model.state.digitalOutput)
      {
        _model.state.minThrottle = (_model.config.output.dshotIdle * 0.1f) + 1001.f;
        _model.state.maxThrottle = 2000.f;
      }
      _model.state.currentMixer = getMixer((MixerType)_model.config.mixerType);
      return 1;
    }

    int update()
    {
      _model.state.stats.start(COUNTER_MIXER);
      updateMixer();
      _model.state.stats.end(COUNTER_MIXER);
      return 1;
    }

  private:
    MixerConfig getMixer(MixerType mixer)
    {
      // default empty mixer
      static MixerEntry mixesEmpty[] = {
        MixerEntry() // terminator
      };

      // quadX mixer
      static MixerEntry mixesQuadX[] = {
        // RR                                     FR                                        RL                                        FL
        MixerEntry(MIXER_SOURCE_ROLL,   0, -100), MixerEntry(MIXER_SOURCE_ROLL,   1, -100), MixerEntry(MIXER_SOURCE_ROLL,   2,  100), MixerEntry(MIXER_SOURCE_ROLL,   3,  100),
        MixerEntry(MIXER_SOURCE_PITCH,  0,  100), MixerEntry(MIXER_SOURCE_PITCH,  1, -100), MixerEntry(MIXER_SOURCE_PITCH,  2,  100), MixerEntry(MIXER_SOURCE_PITCH,  3, -100),
        MixerEntry(MIXER_SOURCE_YAW,    0, -100), MixerEntry(MIXER_SOURCE_YAW,    1,  100), MixerEntry(MIXER_SOURCE_YAW,    2,  100), MixerEntry(MIXER_SOURCE_YAW,    3, -100),
        MixerEntry(MIXER_SOURCE_THRUST, 0,  100), MixerEntry(MIXER_SOURCE_THRUST, 1,  100), MixerEntry(MIXER_SOURCE_THRUST, 2,  100), MixerEntry(MIXER_SOURCE_THRUST, 3,  100),
        MixerEntry() // terminator
      };

      // quadX mixer
      static MixerEntry mixesQuadX1234[] = {
        // FL                                     FR                                        RR                                        RL
        MixerEntry(MIXER_SOURCE_ROLL,   0,  100), MixerEntry(MIXER_SOURCE_ROLL,   1, -100), MixerEntry(MIXER_SOURCE_ROLL,   2, -100), MixerEntry(MIXER_SOURCE_ROLL,   3,  100),
        MixerEntry(MIXER_SOURCE_PITCH,  0, -100), MixerEntry(MIXER_SOURCE_PITCH,  1, -100), MixerEntry(MIXER_SOURCE_PITCH,  2,  100), MixerEntry(MIXER_SOURCE_PITCH,  3,  100),
        MixerEntry(MIXER_SOURCE_YAW,    0, -100), MixerEntry(MIXER_SOURCE_YAW,    1,  100), MixerEntry(MIXER_SOURCE_YAW,    2, -100), MixerEntry(MIXER_SOURCE_YAW,    3,  100),
        MixerEntry(MIXER_SOURCE_THRUST, 0,  100), MixerEntry(MIXER_SOURCE_THRUST, 1,  100), MixerEntry(MIXER_SOURCE_THRUST, 2,  100), MixerEntry(MIXER_SOURCE_THRUST, 3,  100),
        MixerEntry() // terminator
      };

      // tricopter mixer
      static MixerEntry mixesTricopter[] = {
        // FL                                     FR                                        RMotor                                    RServo
        MixerEntry(MIXER_SOURCE_ROLL,   0,    0), MixerEntry(MIXER_SOURCE_ROLL,   1, -100), MixerEntry(MIXER_SOURCE_ROLL,   2,  100), MixerEntry(MIXER_SOURCE_ROLL,   3,    0),
        MixerEntry(MIXER_SOURCE_PITCH,  0,  133), MixerEntry(MIXER_SOURCE_PITCH,  1,  -67), MixerEntry(MIXER_SOURCE_PITCH,  2,  -67), MixerEntry(MIXER_SOURCE_PITCH,  3,    0),
        MixerEntry(MIXER_SOURCE_YAW,    0,    0), MixerEntry(MIXER_SOURCE_YAW,    1,    0), MixerEntry(MIXER_SOURCE_YAW,    2,    0), MixerEntry(MIXER_SOURCE_YAW,    3,  100),
        MixerEntry(MIXER_SOURCE_THRUST, 0,  100), MixerEntry(MIXER_SOURCE_THRUST, 1,  100), MixerEntry(MIXER_SOURCE_THRUST, 2,  100), MixerEntry(MIXER_SOURCE_THRUST, 3,    0),
        MixerEntry() // terminator
      };

      // tricopter mixer
      static MixerEntry mixesGimbal[] = {
        // L                                      R
        MixerEntry(MIXER_SOURCE_PITCH,  0,  100), MixerEntry(MIXER_SOURCE_PITCH,  1,   100),
        MixerEntry(MIXER_SOURCE_YAW,    0,  100), MixerEntry(MIXER_SOURCE_YAW,    1,  -100),
        MixerEntry() // terminator
      };

      switch(mixer)
      {
        case MIXER_QUADX:
          return MixerConfig(4, mixesQuadX);

        case MIXER_QUADX_1234:
          return MixerConfig(4, mixesQuadX1234);

        case MIXER_TRI:
          return MixerConfig(4, mixesTricopter);

        case MIXER_GIMBAL:
          return MixerConfig(2, mixesGimbal);

        case MIXER_CUSTOM:
        case MIXER_CUSTOM_TRI:
        case MIXER_CUSTOM_AIRPLANE:
          return _model.state.customMixer;

        default:
          return MixerConfig(0, mixesEmpty);
      }
      return MixerConfig(0, mixesEmpty);
    }

    void updateMixer()
    {
      float sources[MIXER_SOURCE_MAX];
      sources[MIXER_SOURCE_NULL]   = 0;

      sources[MIXER_SOURCE_ROLL]   = _model.state.output[AXIS_ROLL];
      sources[MIXER_SOURCE_PITCH]  = _model.state.output[AXIS_PITCH];
      sources[MIXER_SOURCE_YAW]    = _model.state.output[AXIS_YAW] * (_model.config.yawReverse ? -1.f : 1.f);
      sources[MIXER_SOURCE_THRUST] = _model.state.output[AXIS_THRUST];

      sources[MIXER_SOURCE_RC_ROLL]   = _model.state.input[AXIS_ROLL];
      sources[MIXER_SOURCE_RC_PITCH]  = _model.state.input[AXIS_PITCH];
      sources[MIXER_SOURCE_RC_YAW]    = _model.state.input[AXIS_YAW];
      sources[MIXER_SOURCE_RC_THRUST] = _model.state.input[AXIS_THRUST];

      sources[MIXER_SOURCE_RC_AUX1] = _model.state.input[AXIS_AUX_1];
      sources[MIXER_SOURCE_RC_AUX2] = _model.state.input[AXIS_AUX_2];
      sources[MIXER_SOURCE_RC_AUX3] = _model.state.input[AXIS_AUX_3];

      float outputs[OUTPUT_CHANNELS];
      for(size_t i = 0; i < OUTPUT_CHANNELS; i++)
      {
        outputs[i] = 0.f;
      }

      const MixerConfig& mixer = _model.state.currentMixer;

      // mix stabilized sources first
      const MixerEntry * entry = mixer.mixes;
      size_t counter = 0;
      while(true)
      {
        if(++counter >= MIXER_RULE_MAX) break;
        if(entry->src == MIXER_SOURCE_NULL) break; // break on terminator
        if(entry->src <= MIXER_SOURCE_YAW && entry->dst < mixer.count && entry->rate != 0)
        {
          outputs[entry->dst] += sources[entry->src] * (entry->rate * 0.01f);
        }
        entry++;
      }

      // airmode logic
      float thrust = sources[MIXER_SOURCE_THRUST];
      if(_model.isActive(MODE_AIRMODE))
      {
        float min = 0.f, max = 0.f;
        for(size_t i = 0; i < OUTPUT_CHANNELS; i++)
        {
          max = std::max(max, outputs[i]);
          min = std::min(min, outputs[i]);
        }
        float range = (max - min) * 0.5f;
        if(range > 1.f)
        {
          for(size_t i = 0; i < OUTPUT_CHANNELS; i++)
          {
            outputs[i] /= range;
          }
          thrust = 0.f;
        }
        else
        {
          thrust = Math::bound(thrust, -1.f + range, 1.f - range);
        }
      }

      // apply other channels
      entry = mixer.mixes;
      counter = 0;
      while(true)
      {
        if(++counter >= MIXER_RULE_MAX) break;
        if(entry->src == MIXER_SOURCE_NULL) break; // break on terminator
        if(entry->dst < mixer.count)
        {
          if(entry->src == MIXER_SOURCE_THRUST)
          {
            outputs[entry->dst] += thrust * (entry->rate * 0.01f);
          }
          else if(entry->src > MIXER_SOURCE_THRUST && entry->src < MIXER_SOURCE_MAX)
          {
            outputs[entry->dst] += sources[entry->src] * (entry->rate * 0.01f);
          }
        }
        entry++;
      }

      writeOutput(outputs, mixer.count);
    }

    void writeOutput(float * out, size_t axes)
    {
      bool stop = _stop();
      for(size_t i = 0; i < OUTPUT_CHANNELS; i++)
      {
        const OutputChannelConfig& och = _model.config.output.channel[i];
        if(i >= axes || stop)
        {
          _model.state.outputUs[i] = och.servo && _model.state.outputDisarmed[i] == 1000 ? och.neutral : _model.state.outputDisarmed[i];
        }
        else
        {
          float v = Math::bound(out[i], -1.f, 1.f);
          if(!_model.state.digitalOutput && och.servo)
          {
            _model.state.outputUs[i] = lrintf(Math::map3(v, -1.f, 0.f, 1.f, och.reverse ? och.max : och.min, och.neutral, och.reverse ? och.min : och.max));
          }
          else
          {
            _model.state.outputUs[i] = lrintf(Math::map(v, -1.f, 1.f, _model.state.minThrottle, _model.state.maxThrottle));
          }
        }
      }
      _write();
    }

    void _write()
    {
      if(!_driver) return;
      for(size_t i = 0; i < OUTPUT_CHANNELS; i++)
      {
        _driver->write(i, _model.state.outputUs[i]);
      }
      _driver->apply();
    }

    bool _stop(void)
    {
      if(!_model.isActive(MODE_ARMED)) return true;
      if(_model.isActive(FEATURE_MOTOR_STOP) && _model.isThrottleLow()) return true;
      return false;
    }

    Model& _model;
    EscDriver * _driver;
};

}

#endif
