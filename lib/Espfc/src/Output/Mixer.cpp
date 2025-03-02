
#include "Mixer.h"
#include "Output/Mixers.h"
#include "platform.h"

namespace Espfc {

namespace Output {

Mixer::Mixer(Model& model): _model(model), _motor(NULL), _servo(NULL) {}

int Mixer::begin()
{
  EscConfig motorConf = {
    .timer = ESC_DRIVER_MOTOR_TIMER,
    .protocol = (EscProtocol)_model.config.output.protocol,
    .rate = _model.config.output.rate,
    .async = !!_model.config.output.async,
    .dshotTelemetry = !!_model.config.output.dshotTelemetry,
  };
  escMotor.begin(motorConf);
  _model.state.mixer.escMotor = _motor = &escMotor;
  _model.logger.info().log(F("MOTOR")).log(FPSTR(EscDriver::getProtocolName((EscProtocol)_model.config.output.protocol))).log(_model.config.output.async).log(_model.config.output.rate).log(_model.config.output.dshotTelemetry).logln(ESC_DRIVER_MOTOR_TIMER);

  if(_model.config.output.servoRate)
  {
    EscConfig servoConf = {
      .timer = ESC_DRIVER_SERVO_TIMER,
      .protocol = ESC_PROTOCOL_PWM,
      .rate = _model.config.output.servoRate,
      .async = true,
      .dshotTelemetry = false,
    };
    escServo.begin(servoConf);
    _model.state.mixer.escServo = _servo = &escServo;
    _model.logger.info().log(F("SERVO")).log(FPSTR(EscDriver::getProtocolName(ESC_PROTOCOL_PWM))).log(true).logln(_model.config.output.servoRate).logln(ESC_DRIVER_SERVO_TIMER);
  }
  _erpmToHz = EscDriver::getErpmToHzRatio(_model.config.output.motorPoles);
  _statsCounterMax = _model.state.mixer.timer.rate / 2;
  _statsCounter = 0;

  for(size_t i = 0; i < OUTPUT_CHANNELS; ++i)
  {
    const OutputChannelConfig& occ = _model.config.output.channel[i];
    if(occ.servo)
    {
      if(_servo)
      {
        _servo->attach(i, _model.config.pin[PIN_OUTPUT_0 + i], 1500);
        _model.logger.info().log(F("SERVO")).log(i).logln(_model.config.pin[PIN_OUTPUT_0 + i]);
      }
    }
    else
    {
      _motor->attach(i, _model.config.pin[PIN_OUTPUT_0 + i], 1000);
      _model.logger.info().log(F("MOTOR")).log(i).logln(_model.config.pin[PIN_OUTPUT_0 + i]);
    }
    _model.state.output.telemetry.errors[i] = 0;
    _model.state.output.telemetry.errorsSum[i] = 0;
    _model.state.output.telemetry.errorsCount[i] = 0;
    _model.state.output.telemetry.temperature[i] = 0;
    _model.state.output.telemetry.current[i] = 0;
    _model.state.output.telemetry.voltage[i] = 0;
    _model.state.output.telemetry.erpm[i] = 0;
    _model.state.output.telemetry.rpm[i] = 0;
    _model.state.output.telemetry.freq[i] = 0;
  }
  motorInitEscDevice(_motor);

  _model.state.mixer.minThrottle = _model.config.output.minThrottle;
  _model.state.mixer.maxThrottle = _model.config.output.maxThrottle;
  _model.state.mixer.digitalOutput = _model.config.output.protocol >= ESC_PROTOCOL_DSHOT150;
  if(_model.state.mixer.digitalOutput)
  {
    _model.state.mixer.minThrottle = (_model.config.output.dshotIdle * 0.1f) + 1001.f;
    _model.state.mixer.maxThrottle = 2000.f;
  }
  _model.state.currentMixer = Mixers::getMixer((MixerType)_model.config.mixer.type, _model.state.customMixer);
  return 1;
}

int FAST_CODE_ATTR Mixer::update()
{
  uint32_t startTime = micros();

  float outputs[OUTPUT_CHANNELS];
  const MixerConfig& mixer = _model.state.currentMixer;

  readTelemetry();
  updateMixer(mixer, outputs);
  writeOutput(mixer, outputs);

  if(_model.config.debug.mode == DEBUG_PIDLOOP)
  {
    _model.state.debug[3] = micros() - startTime;
  }

  _model.state.stats.loopTick();

  if(_model.config.debug.mode == DEBUG_CYCLETIME)
  {
    _model.state.debug[0] = _model.state.stats.loopTime();
    _model.state.debug[1] = lrintf(_model.state.stats.getCpuLoad());
  }

  return 1;
}

void FAST_CODE_ATTR Mixer::updateMixer(const MixerConfig& mixer, float * outputs)
{
  Utils::Stats::Measure mixerMeasure(_model.state.stats, COUNTER_MIXER);

  float sources[MIXER_SOURCE_MAX];
  sources[MIXER_SOURCE_NULL]   = 0;

  sources[MIXER_SOURCE_ROLL]   = _model.state.output.ch[AXIS_ROLL];
  sources[MIXER_SOURCE_PITCH]  = _model.state.output.ch[AXIS_PITCH];
  sources[MIXER_SOURCE_YAW]    = _model.state.output.ch[AXIS_YAW] * (_model.config.mixer.yawReverse ? 1.f : -1.f);
  sources[MIXER_SOURCE_THRUST] = _model.state.output.ch[AXIS_THRUST];

  sources[MIXER_SOURCE_RC_ROLL]   = _model.state.input.ch[AXIS_ROLL];
  sources[MIXER_SOURCE_RC_PITCH]  = _model.state.input.ch[AXIS_PITCH];
  sources[MIXER_SOURCE_RC_YAW]    = _model.state.input.ch[AXIS_YAW];
  sources[MIXER_SOURCE_RC_THRUST] = _model.state.input.ch[AXIS_THRUST];

  for(size_t i = 0; i < 3; i++)
  {
    sources[MIXER_SOURCE_RC_AUX1 + i] = _model.state.input.ch[AXIS_AUX_1 + i];
  }
  
  for(size_t i = 0; i < OUTPUT_CHANNELS; i++)
  {
    outputs[i] = 0.f;
  }

  // mix stabilized sources first
  const MixerEntry * entry = mixer.mixes;
  const MixerEntry * end = mixer.mixes + MIXER_RULE_MAX;
  while(entry != end)
  {
    if(entry->src == MIXER_SOURCE_NULL) break; // break on terminator
    if(entry->src <= MIXER_SOURCE_YAW && entry->dst < mixer.count && entry->rate != 0)
    {
      outputs[entry->dst] += sources[entry->src] * (entry->rate * 0.01f);
    }
    entry++;
  }

  // airmode logic
  float thrust = limitThrust(sources[MIXER_SOURCE_THRUST], (ThrottleLimitType)_model.config.output.throttleLimitType, _model.config.output.throttleLimitPercent);
  if(_model.isAirModeActive())
  {
    float min = 0.f, max = 0.f;
    for(size_t i = 0; i < mixer.count; i++)
    {
      max = std::max(max, outputs[i]);
      min = std::min(min, outputs[i]);
    }
    float range = (max - min) * 0.5f;
    if(range > 1.f)
    {
      for(size_t i = 0; i < mixer.count; i++)
      {
        outputs[i] /= range;
      }
      thrust = 0.f; //limitThrust(0.f);
    }
    else
    {
      thrust = Utils::clamp(thrust, -1.f + range, 1.f - range);
    }
  }

  // apply other channels
  entry = mixer.mixes;
  while(entry != end)
  {
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

  bool saturated = false;
  for(size_t i = 0; i < mixer.count; i++)
  {
    const OutputChannelConfig& occ = _model.config.output.channel[i];
    if(!occ.servo && outputs[i] >= 0.98f) saturated = true;
    outputs[i] = limitOutput(outputs[i], occ, _model.config.output.motorLimit);
  }
  _model.setOutputSaturated(saturated);
}

float FAST_CODE_ATTR Mixer::limitThrust(float thrust, ThrottleLimitType type, int8_t limit)
{
  if(type == THROTTLE_LIMIT_TYPE_NONE || limit >= 100 || limit < 1) return thrust;

  // thrust range is [-1, 1]
  switch(type)
  {
    case THROTTLE_LIMIT_TYPE_SCALE:
      return (thrust + 1.0f) * limit * 0.01f - 1.0f;
    case THROTTLE_LIMIT_TYPE_CLIP:
      return Utils::clamp(thrust, -1.f, (limit * 0.02f) - 1.0f);
    default:
      break;
  }

  return thrust;
}

float FAST_CODE_ATTR Mixer::limitOutput(float output, const OutputChannelConfig& occ, int limit)
{
  if(limit >= 100 || limit < 1) return output;

  if(occ.servo)
  {
    const float factor = limit * 0.01f;
    return Utils::clamp(output, -factor, factor);
  }
  else
  {
    const float factor = limit * 0.02f; // *2
    return Utils::clamp(output + 1.f, 0.f, factor) - 1.0f;
  }
}

void FAST_CODE_ATTR Mixer::writeOutput(const MixerConfig& mixer, float * out)
{
  Utils::Stats::Measure mixerMeasure(_model.state.stats, COUNTER_MIXER_WRITE);

  bool stop = _stop();
  for(size_t i = 0; i < OUTPUT_CHANNELS; i++)
  {
    const OutputChannelConfig& och = _model.config.output.channel[i];
    if(i >= mixer.count || stop)
    {
      _model.state.output.us[i] = och.servo && _model.state.output.disarmed[i] == 1000 ? och.neutral : _model.state.output.disarmed[i];
    }
    else
    {
      if(och.servo)
      {
        const int16_t tmp = lrintf(Utils::map3(out[i], -1.f, 0.f, 1.f, och.reverse ? 2000 : 1000, och.neutral, och.reverse ? 1000 : 2000));
        _model.state.output.us[i] = Utils::clamp(tmp, och.min, och.max);
      }
      else
      {
        float v = Utils::clamp(out[i], -1.f, 1.f);
        _model.state.output.us[i] = lrintf(Utils::map(v, -1.f, 1.f, _model.state.mixer.minThrottle, _model.state.mixer.maxThrottle));
      }
    }
  }

  for(size_t i = 0; i < OUTPUT_CHANNELS; i++)
  {
    const OutputChannelConfig& och = _model.config.output.channel[i];
    if(och.servo)
    {
      if(_servo) _servo->write(i, _model.state.output.us[i]);
    }
    else
    {
      if(_motor) _motor->write(i, _model.state.output.us[i]);
    }
  }

  if(_motor) _motor->apply();
  if(_servo) _servo->apply();
}

void FAST_CODE_ATTR Mixer::readTelemetry()
{
  Utils::Stats::Measure mixerMeasure(_model.state.stats, COUNTER_MIXER_READ);
  if(!_model.config.output.dshotTelemetry || !_motor) return;

  for(size_t i = 0; i < OUTPUT_CHANNELS; i++)
  {
    if(_model.config.output.channel[i].servo) continue;
    uint32_t value = _motor->telemetry(i);
    value = EscDriver::gcrToRawValue(value);

    _model.state.output.telemetry.errorsCount[i]++;

    if(value == EscDriver::INVALID_TELEMETRY_VALUE)
    {
      _model.state.output.telemetry.errorsSum[i]++;
      continue;
    }

    // Decode Extended DSHOT telemetry
    switch (value & 0x0f00)
    {
      case 0x0200:
        // Temperature range (in degree Celsius, just like Blheli_32 and KISS)
        _model.state.output.telemetry.temperature[i] = value & 0x00ff;
        break;
      case 0x0400:
        // Voltage range (0-63,75V step 0,25V)
        _model.state.output.telemetry.voltage[i] = value & 0x00ff;
        break;
      case 0x0600:
        // Current range (0-255A step 1A)
        _model.state.output.telemetry.current[i] = value & 0x00ff;
        break;
      case 0x0800:
        // Debug 1 value
        _model.state.output.telemetry.debug1[i] = value & 0x00ff;
        break;
      case 0x0A00:
        // Debug 2 value
        _model.state.output.telemetry.debug2[i] = value & 0x00ff;
        break;
      case 0x0C00:
        // Debug 3 value
        _model.state.output.telemetry.debug3[i] = value & 0x00ff;
        break;
      case 0x0E00:
        // State / Events
        _model.state.output.telemetry.events[i] = value & 0x00ff;
        break;
      default:
        value = EscDriver::convertToValue(value);
        _model.state.output.telemetry.erpm[i] = EscDriver::convertToErpm(value);
        _model.state.output.telemetry.rpm[i] = erpmToRpm(_model.state.output.telemetry.erpm[i]);
        _model.setDebug(DEBUG_DSHOT_RPM_TELEMETRY, i, _model.state.output.telemetry.erpm[i]);
        break;
    }
  }

  for(size_t i = 0; i < OUTPUT_CHANNELS; i++)
  {
    _model.state.output.telemetry.freq[i] = _model.state.gyro.rpmFreqFilter[i].update(erpmToHz(_model.state.output.telemetry.erpm[i]));
  }

  _statsCounter++;
  if(_statsCounter >= _statsCounterMax)
  {
    for(size_t i = 0; i < OUTPUT_CHANNELS; i++)
    {
      if(_model.state.output.telemetry.errorsCount[i])
      {
        _model.state.output.telemetry.errors[i] = _model.state.output.telemetry.errorsSum[i] * 10000 / _model.state.output.telemetry.errorsCount[i];
        if(_model.config.debug.mode == DEBUG_DSHOT_RPM_ERRORS)
        {
          _model.state.debug[i] = _model.state.output.telemetry.errors[i];
        }
      }
      _model.state.output.telemetry.errorsCount[i] = 0;
      _model.state.output.telemetry.errorsSum[i] = 0;
    }
    _statsCounter = 0;
  }
}

float inline Mixer::erpmToHz(float erpm)
{
  return _erpmToHz * erpm;
}

float inline Mixer::erpmToRpm(float erpm)
{
  return erpmToHz(erpm) * EscDriver::SECONDS_PER_MINUTE;
}

bool Mixer::_stop(void)
{
  if(!_model.isModeActive(MODE_ARMED)) return true;
  if(_model.isFeatureActive(FEATURE_MOTOR_STOP) && _model.isThrottleLow()) return true;
  return false;
}

}

}

