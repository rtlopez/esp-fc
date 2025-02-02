
#include "Input.h"
#include "Math/Utils.h"
#include "Utils/MemoryHelper.h"

namespace Espfc {

Input::Input(Model& model): _model(model) {}

int Input::begin()
{
  _device = getInputDevice();
  _model.state.inputChannelCount = _device ? _device->getChannelCount() : INPUT_CHANNELS;
  _model.state.inputFrameDelta = FRAME_TIME_DEFAULT_US;
  _model.state.inputFrameRate = 1000000ul / _model.state.inputFrameDelta;
  _model.state.inputFrameCount = 0;
  _model.state.inputAutoFactor = 1.f / (2.f + _model.config.input.filterAutoFactor * 0.1f);
  switch(_model.config.input.interpolationMode)
  {
    case INPUT_INTERPOLATION_AUTO:
      _model.state.inputInterpolationDelta = Math::clamp(_model.state.inputFrameDelta, (uint32_t)4000, (uint32_t)40000) * 0.000001f; // estimate real interval
      break;
    case INPUT_INTERPOLATION_MANUAL:
      _model.state.inputInterpolationDelta = _model.config.input.interpolationInterval * 0.001f; // manual interval
      break;
    case INPUT_INTERPOLATION_DEFAULT:
    case INPUT_INTERPOLATION_OFF:
    default:
      _model.state.inputInterpolationDelta = FRAME_TIME_DEFAULT_US * 0.000001f;
      break;
  }
  _model.state.inputInterpolationStep = _model.state.loopTimer.intervalf / _model.state.inputInterpolationDelta;
  _step = 0.0f;
  for(size_t c = 0; c < INPUT_CHANNELS; ++c)
  {
    if(_device) _filter[c].begin(FilterConfig(_device->needAverage() ? FILTER_FIR2 : FILTER_NONE, 1), _model.state.loopTimer.rate);
    int16_t v = c == AXIS_THRUST ? PWM_RANGE_MIN : PWM_RANGE_MID;
    _model.state.inputRaw[c] = v;
    _model.state.inputBuffer[c] = v;
    _model.state.inputBufferPrevious[c] = v;
    setInput((Axis)c, v, true, true);
  }
  return 1;
}

int16_t FAST_CODE_ATTR Input::getFailsafeValue(uint8_t c)
{
  const InputChannelConfig& ich = _model.config.input.channel[c];
  switch(ich.fsMode)
  {
    case FAILSAFE_MODE_AUTO:
      return c == AXIS_THRUST ? PWM_RANGE_MIN : PWM_RANGE_MID;
    case FAILSAFE_MODE_SET:
      return ich.fsValue;
    case FAILSAFE_MODE_INVALID:
    case FAILSAFE_MODE_HOLD:
    default:
      return _model.state.inputBuffer[c];
  }
}

void FAST_CODE_ATTR Input::setInput(Axis i, float v, bool newFrame, bool noFilter)
{
  const InputChannelConfig& ich = _model.config.input.channel[i];
  if(i <= AXIS_THRUST)
  {
    const float nv = noFilter ? v : _model.state.inputFilter[i].update(v);
    _model.state.inputUs[i] = nv;
    _model.state.input[i] = Math::map(nv, ich.min, ich.max, -1.f, 1.f);
  }
  else if(newFrame)
  {
    _model.state.inputUs[i] = v;
    _model.state.input[i] = Math::map(v, ich.min, ich.max, -1.f, 1.f);
  }
}

int FAST_CODE_ATTR Input::update()
{
  if(!_device) return 0;

  uint32_t startTime = micros();

  InputStatus status = readInputs();

  if(!failsafe(status))
  {
    filterInputs(status);
  }

  if(_model.config.debugMode == DEBUG_PIDLOOP)
  {
    _model.state.debug[1] = micros() - startTime;
  }

  return 1;
}

InputStatus FAST_CODE_ATTR Input::readInputs()
{
  Stats::Measure readMeasure(_model.state.stats, COUNTER_INPUT_READ);
  uint32_t startTime = micros();

  InputStatus status = _device->update();

  if(_model.config.debugMode == DEBUG_RX_TIMING)
  {
    _model.state.debug[0] = micros() - startTime;
  }

  if(status == INPUT_IDLE) return status;

  _model.state.inputRxLoss = (status == INPUT_LOST || status == INPUT_FAILSAFE);
  _model.state.inputRxFailSafe = (status == INPUT_FAILSAFE);
  _model.state.inputFrameCount++;

  updateFrameRate();

  processInputs();

  if(_model.config.debugMode == DEBUG_RX_SIGNAL_LOSS)
  {
    _model.state.debug[0] = !_model.state.inputRxLoss;
    _model.state.debug[1] = _model.state.inputRxFailSafe;
    _model.state.debug[2] = _model.state.inputChannelsValid;
    _model.state.debug[3] = _model.state.inputLossTime / (100 * 1000);
  }

  return status;
}

void FAST_CODE_ATTR Input::processInputs()
{
  if(_model.state.inputFrameCount < 5) return; // ignore few first frames that might be garbage

  uint32_t startTime = micros();

  uint16_t channels[INPUT_CHANNELS];
  _device->get(channels, _model.state.inputChannelCount);

  _model.state.inputChannelsValid = true;
  for(size_t c = 0; c < _model.state.inputChannelCount; c++)
  {
    const InputChannelConfig& ich = _model.config.input.channel[c];

    // remap channels
    int16_t v = _model.state.inputRaw[c] = (int16_t)channels[ich.map];

    // adj midrc
    v -= _model.config.input.midRc - PWM_RANGE_MID;

    // adj range
    //float t = Math::map3((float)v, (float)ich.min, (float)ich.neutral, (float)ich.max, (float)PWM_RANGE_MIN, (float)PWM_RANGE_MID, (float)PWM_RANGE_MAX);
    float t = Math::mapi(v, ich.min, ich.max, PWM_RANGE_MIN, PWM_RANGE_MAX);

    // filter if required
    t = _filter[c].update(t);
    v = lrintf(t);

    // apply deadband
    if(c < AXIS_THRUST)
    {
      v = Math::deadband(v - PWM_RANGE_MID, (int)_model.config.input.deadband) + PWM_RANGE_MID;
    }

    // check if inputs are valid, apply failsafe value otherwise
    if(v < _model.config.input.minRc || v > _model.config.input.maxRc)
    {
      v = getFailsafeValue(c);
      if(c <= AXIS_THRUST) _model.state.inputChannelsValid = false;
    }

    // update input buffer
    _model.state.inputBufferPrevious[c] = _model.state.inputBuffer[c];
    _model.state.inputBuffer[c] = v;
  }

  if(_model.config.debugMode == DEBUG_RX_TIMING)
  {
    _model.state.debug[2] = micros() - startTime;
  }
}

bool FAST_CODE_ATTR Input::failsafe(InputStatus status)
{
  Stats::Measure readMeasure(_model.state.stats, COUNTER_FAILSAFE);

  if(_model.isSwitchActive(MODE_FAILSAFE))
  {
    failsafeStage2();
    return false; // not real failsafe, rx link is still valid
  }

  if(status == INPUT_RECEIVED)
  {
    failsafeIdle();
    return false;
  }

  if(status == INPUT_FAILSAFE)
  {
    failsafeStage2();
    return true;
  }

  // stage 2 timeout
  _model.state.inputLossTime = micros() - _model.state.inputFrameTime;
  if(_model.state.inputLossTime > Math::clamp((uint32_t)_model.config.failsafe.delay, (uint32_t)2u, (uint32_t)200u) * TENTH_TO_US)
  {
    failsafeStage2();
    return true;
  }

  // stage 1 timeout (100ms)
  if(_model.state.inputLossTime >= 2 * TENTH_TO_US)
  {
    failsafeStage1();
    return true;
  }

  return false;
}

void FAST_CODE_ATTR Input::failsafeIdle()
{
  _model.state.failsafe.phase = FC_FAILSAFE_IDLE;
  _model.state.inputLossTime = 0;
}

void FAST_CODE_ATTR Input::failsafeStage1()
{
  _model.state.failsafe.phase = FC_FAILSAFE_RX_LOSS_DETECTED;
  _model.state.inputRxLoss = true;
  for(size_t i = 0; i < _model.state.inputChannelCount; i++)
  {
    setInput((Axis)i, getFailsafeValue(i), true, true);
  }
}

void FAST_CODE_ATTR Input::failsafeStage2()
{
  _model.state.failsafe.phase = FC_FAILSAFE_RX_LOSS_DETECTED;
  _model.state.inputRxLoss = true;
  _model.state.inputRxFailSafe = true;
  if(_model.isModeActive(MODE_ARMED))
  {
    _model.state.failsafe.phase = FC_FAILSAFE_LANDED;
    _model.disarm(DISARM_REASON_FAILSAFE);
  }
}

void FAST_CODE_ATTR Input::filterInputs(InputStatus status)
{
  Stats::Measure filterMeasure(_model.state.stats, COUNTER_INPUT_FILTER);
  uint32_t startTime = micros();

  const bool newFrame = status != INPUT_IDLE;
  const bool interpolation = _model.config.input.interpolationMode != INPUT_INTERPOLATION_OFF && _model.config.input.filterType == INPUT_INTERPOLATION;

  if(interpolation)
  {
    if(newFrame)
    {
      _step = 0.0f;
    }
    if(_step < 1.f)
    {
      _step += _model.state.inputInterpolationStep;
    }
  }

  for(size_t c = 0; c < _model.state.inputChannelCount; c++)
  {
    float v = _model.state.inputBuffer[c];
    if(c <= AXIS_THRUST)
    {
      v = interpolation ? _interpolate(_model.state.inputBufferPrevious[c], v, _step) : v;
    }
    setInput((Axis)c, v, newFrame);
  }

  if(_model.config.debugMode == DEBUG_RX_TIMING)
  {
    _model.state.debug[3] = micros() - startTime;
  }
}

void FAST_CODE_ATTR Input::updateFrameRate()
{
  const uint32_t now = micros();
  const uint32_t frameDelta = now - _model.state.inputFrameTime;

  _model.state.inputFrameTime = now;
  _model.state.inputFrameDelta += (((int)frameDelta - (int)_model.state.inputFrameDelta) >> 3); // avg * 0.125
  _model.state.inputFrameRate = 1000000ul / _model.state.inputFrameDelta;

  if (_model.config.input.interpolationMode == INPUT_INTERPOLATION_AUTO && _model.config.input.filterType == INPUT_INTERPOLATION)
  {
    _model.state.inputInterpolationDelta = Math::clamp(_model.state.inputFrameDelta, (uint32_t)4000, (uint32_t)40000) * 0.000001f; // estimate real interval
    _model.state.inputInterpolationStep = _model.state.loopTimer.intervalf / _model.state.inputInterpolationDelta;
  }

  if(_model.config.debugMode == DEBUG_RC_SMOOTHING_RATE)
  {
    _model.state.debug[0] = _model.state.inputFrameDelta / 10;
    _model.state.debug[1] = _model.state.inputFrameRate;
  }

  // auto cutoff input freq
  float freq = std::max(_model.state.inputFrameRate * _model.state.inputAutoFactor, 15.f); // no lower than 15Hz
  if(freq > _model.state.inputAutoFreq * 1.1f || freq < _model.state.inputAutoFreq * 0.9f)
  {
    _model.state.inputAutoFreq += 0.25f * (freq - _model.state.inputAutoFreq);
    if(_model.config.debugMode == DEBUG_RC_SMOOTHING_RATE)
    {
      _model.state.debug[2] = lrintf(freq);
      _model.state.debug[3] = lrintf(_model.state.inputAutoFreq);
    }
    FilterConfig conf((FilterType)_model.config.input.filter.type, _model.state.inputAutoFreq);
    FilterConfig confDerivative((FilterType)_model.config.input.filterDerivative.type, _model.state.inputAutoFreq);
    for(size_t i = 0; i <= AXIS_THRUST; i++)
    {
      if(_model.config.input.filter.freq == 0)
      {
        _model.state.inputFilter[i].reconfigure(conf, _model.state.loopTimer.rate);
      }
      if(_model.config.input.filterDerivative.freq == 0)
      {
        _model.state.innerPid[i].ftermFilter.reconfigure(confDerivative, _model.state.loopTimer.rate);
      }
    }
  }

  if(_model.config.debugMode == DEBUG_RX_TIMING)
  {
    _model.state.debug[1] = micros() - now;
  }
}

Device::InputDevice * Input::getInputDevice()
{
  Device::SerialDevice * serial = _model.getSerialStream(SERIAL_FUNCTION_RX_SERIAL);
  if(serial && _model.isFeatureActive(FEATURE_RX_SERIAL))
  {
    switch(_model.config.input.serialRxProvider)
    {
      case SERIALRX_IBUS:
        _ibus.begin(serial);
        _model.logger.info().logln(F("RX IBUS"));
        return &_ibus;

      case SERIALRX_SBUS:
        _sbus.begin(serial);
        _model.logger.info().logln(F("RX SBUS"));
        return &_sbus;

      case SERIALRX_CRSF:
        _crsf.begin(serial);
        _model.logger.info().logln(F("RX CRSF"));
        return &_crsf;
    }
  }
  else if(_model.isFeatureActive(FEATURE_RX_PPM) && _model.config.pin[PIN_INPUT_RX] != -1)
  {
    _ppm.begin(_model.config.pin[PIN_INPUT_RX], _model.config.input.ppmMode);
    _model.logger.info().log(F("RX PPM")).log(_model.config.pin[PIN_INPUT_RX]).logln(_model.config.input.ppmMode);
    return &_ppm;
  }
#if defined(ESPFC_ESPNOW)
  else if(_model.isFeatureActive(FEATURE_RX_SPI))
  {
    int status = _espnow.begin();
    _model.logger.info().log(F("RX ESPNOW")).logln(status);
    return &_espnow;
  }
#endif

  return nullptr;
}

}
