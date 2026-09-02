
#include "Input.h"
#include "ModelConfig.h"
#include "Utils/Filter.h"
#include "Utils/Math.hpp"
#include "Utils/MemoryHelper.h"

namespace Espfc {

Input::Input(Model& model, TelemetryManager& telemetry): _model(model), _telemetry(telemetry) {}

int Input::begin()
{
  _device = getInputDevice();
  _model.state.input.channelCount = _device ? _device->getChannelCount() : INPUT_CHANNELS;
  _model.state.input.frameDelta = FRAME_TIME_DEFAULT_US;
  _model.state.input.frameRate = 1000000ul / _model.state.input.frameDelta;
  _model.state.input.frameCount = 0;

  reload(MODEL_CHANGE_INPUT);

  for (size_t c = 0; c < INPUT_CHANNELS; ++c)
  {
    const int16_t v = c == AXIS_THRUST ? PWM_RANGE_MIN : PWM_RANGE_MID;
    _model.state.input.raw[c] = v;
    _model.state.input.buffer[c] = v;
    _model.state.input.bufferPrevious[c] = v;
    setInput((Axis)c, v, true, true);
  }
  return 1;
}

int Input::reload(ModelChangeEvent event)
{
  switch (event)
  {
    case MODEL_CHANGE_INPUT: {
      _model.state.input.autoFactor = 1.f / (2.f + _model.config.input.filterAutoFactor * 0.1f);
      _model.state.input.autoThrottleFactor = 1.f / (2.f + _model.config.input.filterAutoThrottleFactor * 0.1f);
      const FilterConfig rxFilter{_device && _device->needAverage() ? FILTER_FIR2 : FILTER_NONE, 1};
      const FilterConfig inputFilter{_model.config.input.filterEnable ? _model.config.input.filter
                                                                      : FilterConfig(FILTER_PT3, 25)};
      const FilterConfig throtleFilter{_model.config.input.filterEnable ? _model.config.input.filterThrottle
                                                                        : FilterConfig(FILTER_PT3, 25)};
      for (size_t i = 0; i < AXIS_COUNT_RPYT; i++)
      {
        _filter[i].begin(rxFilter, 100); // rx filter uses FIR2 on NONE, sample rate doesn't really matter here
        if (i == AXIS_THRUST)
        {
          _model.state.input.filter[i].begin(throtleFilter, _model.state.input.timer.rate);
        }
        else
        {
          _model.state.input.filter[i].begin(inputFilter, _model.state.input.timer.rate);
        }
      }
      break;
    }
    default:
      break;
  }
  return 1;
}

int16_t FAST_CODE_ATTR Input::getFailsafeValue(uint8_t c)
{
  const InputChannelConfig& ich = _model.config.input.channel[c];
  switch (ich.fsMode)
  {
    case FAILSAFE_MODE_AUTO:
      return c == AXIS_THRUST ? PWM_RANGE_MIN : PWM_RANGE_MID;
    case FAILSAFE_MODE_SET:
      return ich.fsValue;
    case FAILSAFE_MODE_INVALID:
    case FAILSAFE_MODE_HOLD:
    default:
      return _model.state.input.buffer[c];
  }
}

void FAST_CODE_ATTR Input::setInput(Axis i, float v, bool newFrame, bool noFilter)
{
  const InputChannelConfig& ich = _model.config.input.channel[i];
  if (i <= AXIS_THRUST)
  {
    const float nv = noFilter ? v : _model.state.input.filter[i].update(v);
    _model.state.input.us[i] = nv;
    _model.state.input.ch[i] = Utils::map(nv, ich.min, ich.max, -1.f, 1.f);
  }
  else if (newFrame)
  {
    _model.state.input.us[i] = v;
    _model.state.input.ch[i] = Utils::map(v, ich.min, ich.max, -1.f, 1.f);
  }
}

int FAST_CODE_ATTR Input::update()
{
  if (!_device) return 0;

  uint32_t startTime = micros();

  InputStatus status = readInputs();

  if (!failsafe(status))
  {
    filterInputs(status);
  }

  if (_model.config.debug.mode == DEBUG_PIDLOOP)
  {
    _model.state.debug[1] = micros() - startTime;
  }

  return 1;
}

InputStatus FAST_CODE_ATTR Input::readInputs()
{
  Utils::Stats::Measure measure(_model.state.stats, COUNTER_INPUT_READ);
  uint32_t startTime = micros();

  InputStatus status = _device->update();

  if (_model.config.debug.mode == DEBUG_RX_TIMING)
  {
    _model.state.debug[0] = micros() - startTime;
  }

  if (status == INPUT_IDLE) return status;

  _model.state.input.rxLoss = (status == INPUT_LOST || status == INPUT_FAILSAFE);
  _model.state.input.rxFailSafe = (status == INPUT_FAILSAFE);
  _model.state.input.frameCount++;

  updateFrameRate();

  processInputs();

  if (_model.config.debug.mode == DEBUG_RX_SIGNAL_LOSS)
  {
    _model.state.debug[0] = !_model.state.input.rxLoss;
    _model.state.debug[1] = _model.state.input.rxFailSafe;
    _model.state.debug[2] = _model.state.input.channelsValid;
    _model.state.debug[3] = _model.state.input.lossTime / (100 * 1000);
  }

  return status;
}

void FAST_CODE_ATTR Input::processInputs()
{
  if (_model.state.input.frameCount < 5) return; // ignore few first frames that might be garbage

  uint32_t startTime = micros();

  uint16_t channels[INPUT_CHANNELS];
  _device->get(channels, _model.state.input.channelCount);

  _model.state.input.channelsValid = true;
  for (size_t c = 0; c < _model.state.input.channelCount; c++)
  {
    const InputChannelConfig& ich = _model.config.input.channel[c];

    // remap channels
    int16_t v = _model.state.input.raw[c] = (int16_t)channels[ich.map];

    // adj midrc
    v -= _model.config.input.midRc - PWM_RANGE_MID;

    // adj range
    // float t = Utils::map3((float)v, (float)ich.min, (float)ich.neutral, (float)ich.max, (float)PWM_RANGE_MIN,
    // (float)PWM_RANGE_MID, (float)PWM_RANGE_MAX);
    float t = Utils::mapi(v, ich.min, ich.max, PWM_RANGE_MIN, PWM_RANGE_MAX);

    // filter if required
    t = _filter[c].update(t);
    v = lrintf(t);

    // apply deadband
    if (c < AXIS_THRUST)
    {
      v = Utils::deadband(v - PWM_RANGE_MID, (int)_model.config.input.deadband) + PWM_RANGE_MID;
    }

    // check if inputs are valid, apply failsafe value otherwise
    if (v < _model.config.input.minRc || v > _model.config.input.maxRc)
    {
      v = getFailsafeValue(c);
      if (c <= AXIS_THRUST) _model.state.input.channelsValid = false;
    }

    // update input buffer
    _model.state.input.bufferPrevious[c] = _model.state.input.buffer[c];
    _model.state.input.buffer[c] = v;
  }

  if (_model.config.debug.mode == DEBUG_RX_TIMING)
  {
    _model.state.debug[2] = micros() - startTime;
  }
}

bool FAST_CODE_ATTR Input::failsafe(InputStatus status)
{
  Utils::Stats::Measure measure(_model.state.stats, COUNTER_FAILSAFE);

  if (_model.isSwitchActive(MODE_FAILSAFE))
  {
    failsafeStage2();
    return false; // not real failsafe, rx link is still valid
  }

  if (status == INPUT_RECEIVED)
  {
    failsafeIdle();
    return false;
  }

  if (status == INPUT_FAILSAFE)
  {
    failsafeStage2();
    return true;
  }

  // stage 2 timeout
  _model.state.input.lossTime = micros() - _model.state.input.frameTime;
  if (_model.state.input.lossTime > std::clamp<uint32_t>(_model.config.failsafe.delay, 2u, 200u) * TENTH_TO_US)
  {
    failsafeStage2();
    return true;
  }

  // stage 1 timeout (100ms)
  if (_model.state.input.lossTime >= 2 * TENTH_TO_US)
  {
    failsafeStage1();
    return true;
  }

  return false;
}

void FAST_CODE_ATTR Input::failsafeIdle()
{
  _model.state.failsafe.phase = FC_FAILSAFE_IDLE;
  _model.state.input.lossTime = 0;
}

void FAST_CODE_ATTR Input::failsafeStage1()
{
  _model.state.failsafe.phase = FC_FAILSAFE_RX_LOSS_DETECTED;
  _model.state.input.rxLoss = true;
  for (size_t i = 0; i < _model.state.input.channelCount; i++)
  {
    setInput((Axis)i, getFailsafeValue(i), true, true);
  }
}

void FAST_CODE_ATTR Input::failsafeStage2()
{
  _model.state.failsafe.phase = FC_FAILSAFE_RX_LOSS_DETECTED;
  _model.state.input.rxLoss = true;
  _model.state.input.rxFailSafe = true;
  if (_model.isModeActive(MODE_ARMED))
  {
    _model.state.failsafe.phase = FC_FAILSAFE_LANDED;
    _model.disarm(DISARM_REASON_FAILSAFE);
  }
}

void FAST_CODE_ATTR Input::filterInputs(InputStatus status)
{
  Utils::Stats::Measure measure(_model.state.stats, COUNTER_INPUT_FILTER);
  uint32_t startTime = micros();

  const bool newFrame = status != INPUT_IDLE;

  for (size_t c = 0; c < _model.state.input.channelCount; c++)
  {
    const float v = _model.state.input.buffer[c];
    setInput((Axis)c, v, newFrame);
  }

  if (_model.config.debug.mode == DEBUG_RX_TIMING)
  {
    _model.state.debug[3] = micros() - startTime;
  }
}

void FAST_CODE_ATTR Input::updateFrameRate()
{
  auto& input = _model.state.input;
  const uint32_t now = micros();
  const uint32_t frameDelta = now - input.frameTime;

  input.frameTime = now;
  input.frameDelta += (((int)frameDelta - (int)input.frameDelta) >> 3); // avg * 0.125
  input.frameRate = 1000000ul / input.frameDelta;

  if (_model.config.debug.mode == DEBUG_RC_SMOOTHING_RATE)
  {
    _model.state.debug[0] = input.frameDelta / 10;
    _model.state.debug[1] = input.frameRate;
  }

  // auto cutoff input freq
  float freq = std::clamp(input.frameRate * input.autoFactor, 15.f, 500.f);                 // no lower than 15Hz
  float throttleFreq = std::clamp(input.frameRate * input.autoThrottleFactor, 15.f, 500.f); // no lower than 15Hz
  if (freq > input.autoFreq * 1.1f || freq < input.autoFreq * 0.9f)
  {
    input.autoFreq += 0.25f * (freq - input.autoFreq);                         // lpf
    input.autoThrottleFreq += 0.25f * (throttleFreq - input.autoThrottleFreq); // lpf

    FilterConfig conf{(FilterType)_model.config.input.filter.type, std::clamp<int16_t>(input.autoFreq, 15, 500)};
    FilterConfig confThrottle{(FilterType)_model.config.input.filterThrottle.type,
                              std::clamp<int16_t>(input.autoThrottleFreq, 15, 500)};
    FilterConfig confDerivative{(FilterType)_model.config.input.filterDerivative.type,
                                std::clamp<int16_t>(input.autoFreq, 15, 500)};

    for (size_t i = 0; i < AXIS_COUNT_RPY; i++)
    {
      if (_model.config.input.filter.freq == 0)
      {
        _model.state.input.filter[i].reconfigure(conf, _model.state.loopTimer.rate);
      }
      if (_model.config.input.filterDerivative.freq == 0)
      {
        _model.state.innerPid[i].ftermFilter.reconfigure(confDerivative, _model.state.loopTimer.rate);
      }
    }

    if (_model.config.input.filterThrottle.freq == 0)
    {
      _model.state.input.filter[AXIS_THRUST].reconfigure(confThrottle, _model.state.loopTimer.rate);
    }

    if (_model.config.debug.mode == DEBUG_RC_SMOOTHING_RATE)
    {
      _model.state.debug[2] = lrintf(freq);
      _model.state.debug[3] = lrintf(input.autoFreq);
      _model.state.debug[4] = lrintf(input.autoThrottleFreq);
    }
  }

  if (_model.config.debug.mode == DEBUG_RX_TIMING)
  {
    _model.state.debug[1] = micros() - now;
  }
}

Device::InputDevice* Input::getInputDevice()
{
  auto* serial = _model.getSerialStream(SERIAL_FUNCTION_RX_SERIAL);
  if (serial && _model.isFeatureActive(FEATURE_RX_SERIAL))
  {
    switch (_model.config.input.serialRxProvider)
    {
      case SERIALRX_IBUS:
        _ibus.begin(serial);
        _model.logger.info().logln("RX IBUS");
        return &_ibus;

      case SERIALRX_SBUS:
        _sbus.begin(serial);
        _model.logger.info().logln("RX SBUS");
        return &_sbus;

      case SERIALRX_CRSF:
        _crsf.begin(serial, _model.isFeatureActive(FEATURE_TELEMETRY) ? &_telemetry : nullptr);
        _model.logger.info().logln("RX CRSF");
        return &_crsf;
    }
  }
  else if (_model.isFeatureActive(FEATURE_RX_PPM) && _model.config.pin[PIN_INPUT_RX] != -1)
  {
    _ppm.begin(_model.config.pin[PIN_INPUT_RX], _model.config.input.ppmMode);
    _model.logger.info().log("RX PPM").log(_model.config.pin[PIN_INPUT_RX]).logln(_model.config.input.ppmMode);
    return &_ppm;
  }
#if defined(ESPFC_ESPNOW)
  else if (_model.isFeatureActive(FEATURE_RX_SPI))
  {
    int status = _espnow.begin();
    _model.logger.info().log("RX ESPNOW").logln(status);
    return &_espnow;
  }
#endif

  return nullptr;
}

} // namespace Espfc
