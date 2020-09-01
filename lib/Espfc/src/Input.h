#ifndef _ESPFC_INPUT_H_
#define _ESPFC_INPUT_H_

#include "Model.h"
#include "Math/Utils.h"
#include "Device/InputDevice.h"
#include "Device/InputPPM.h"
#include "Device/InputSBUS.h"

namespace Espfc {

enum FailsafeChannelMode {
  FAILSAFE_MODE_AUTO,
  FAILSAFE_MODE_HOLD,
  FAILSAFE_MODE_SET,
  FAILSAFE_MODE_INVALID
};

enum InputPwmRange {
  PWM_RANGE_MIN = 1000,
  PWM_RANGE_MID = 1500,
  PWM_RANGE_MAX = 2000
};

class Input
{
  public:
    Input(Model& model): _model(model) {}

    int begin()
    {
      _device = getInputDevice();
      _model.state.inputChannelCount = _device ? _device->getChannelCount() : INPUT_CHANNELS;
      _model.state.inputFrameDelta = FRAME_TIME_DEFAULT_US;
      _model.state.inputFrameRate = 1000000ul / _model.state.inputFrameDelta;
      _model.state.inputFrameCount = 0;
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

    int16_t getFailsafeValue(uint8_t c)
    {
      const InputChannelConfig& ich = _model.config.input.channel[c];
      switch(ich.fsMode)
      {
        case FAILSAFE_MODE_AUTO:
          return c != AXIS_THRUST ? _model.config.input.midRc : _model.config.input.minRc;
        case FAILSAFE_MODE_SET:
          return ich.fsValue;
        case FAILSAFE_MODE_INVALID:
        case FAILSAFE_MODE_HOLD:
        default:
          return _model.state.inputBuffer[c];
      }
    }

    void setInput(Axis i, float v, bool newFrame, bool noDelta = false)
    {
      _model.state.inputUs[i] = v;
      const InputChannelConfig& ich = _model.config.input.channel[i];
      if(i <= AXIS_THRUST) {
        _model.state.input[i] = Math::map(_model.state.inputUs[i], ich.min, ich.max, -1.f, 1.f);
        _model.state.inputDelta[i] = noDelta ? 0 : (_model.state.input[i] - _model.state.inputPrevious[i]) * _model.state.loopTimer.rate;
        _model.state.inputPrevious[i] = _model.state.input[i];
      } else if(newFrame) {
        _model.state.input[i] = Math::map(_model.state.inputUs[i], ich.min, ich.max, -1.f, 1.f);
        _model.state.inputDelta[i] = noDelta ? 0 : (_model.state.input[i] - _model.state.inputPrevious[i]) * _model.state.inputFrameRate;
        _model.state.inputPrevious[i] = _model.state.input[i];
      }
    }

    int update()
    {
      if(!_device) return 0;

      InputStatus status = readInput();

      failsafe(status);

      filterInput(status);

      return 1;
    }

    InputStatus readInput()
    {
      Stats::Measure readMeasure(_model.state.stats, COUNTER_INPUT_READ);
      InputStatus status = _device->update();

      if(status == INPUT_IDLE) return status;

      _model.state.inputRxLoss = (status == INPUT_LOST || status == INPUT_FAILSAFE);
      _model.state.inputRxFailSafe = (status == INPUT_FAILSAFE);
      _model.state.inputFrameCount++;

      updateFrameRate(micros());

      processInput();

      if(_model.config.debugMode == DEBUG_RX_SIGNAL_LOSS)
      {
        _model.state.debug[0] = !_model.state.inputRxLoss;
        _model.state.debug[1] = _model.state.inputRxFailSafe;
        _model.state.debug[2] = _model.state.inputChannelsValid;
        _model.state.debug[3] = _model.state.inputRaw[AXIS_THRUST];
      }

      return status;
    }

    void processInput()
    {
      if(_model.state.inputFrameCount < 5) return; // ignore few first frames that might be garbage

      _model.state.inputChannelsValid = true;
      for(size_t c = 0; c < _model.state.inputChannelCount; c++)
      {
        const InputChannelConfig& ich = _model.config.input.channel[c];

        // remap channels
        int16_t v = _model.state.inputRaw[c] = _device->get(ich.map);

        // adj midrc
        v -= _model.config.input.midRc - PWM_RANGE_MID;

        // adj range
        float t = Math::map3((float)v, (float)ich.min, (float)ich.neutral, (float)ich.max, (float)PWM_RANGE_MIN, (float)PWM_RANGE_MID, (float)PWM_RANGE_MAX);

        // filter if required
        t = _filter[c].update(t);
        v = lrintf(t);

        // apply deadband
        if(c < AXIS_THRUST)
        {
          v = Math::deadband(v - PWM_RANGE_MID, (int)_model.config.input.deadband) + PWM_RANGE_MID;
        }

        // check if inputs are valid then apply failsafe value
        if(v < _model.config.input.minRc || v > _model.config.input.maxRc)
        {
          v = getFailsafeValue(c);
          if(c <= AXIS_THRUST) _model.state.inputChannelsValid = false;
        }

        // update input buffer
        _model.state.inputBufferPrevious[c] = _model.state.inputBuffer[c];
        _model.state.inputBuffer[c] = v;
      }
    }

    void failsafe(InputStatus status)
    {
      Stats::Measure readMeasure(_model.state.stats, COUNTER_FAILSAFE);

      if(_model.isSwitchActive(MODE_FAILSAFE))
      {
        failsafeStage2();
        return;
      }

      if(status == INPUT_RECEIVED)
      {
        failsafeIdle();
        return;
      }

      if(status == INPUT_FAILSAFE)
      {
        failsafeStage2();
        return;
      }

      uint32_t lossTime = micros() - _model.state.inputFrameTime;
      if(lossTime >= Math::clamp((uint32_t)_model.config.failsafe.delay, 1u, 200u) * TENTH_TO_US)
      {
        failsafeStage2();
        return;
      }
      else if(lossTime >= 1 * TENTH_TO_US)
      {
        failsafeStage1();
        return;
      }
    }

    void failsafeIdle()
    {
      _model.state.failsafe.phase = FAILSAFE_IDLE;
    }

    void failsafeStage1()
    {
      _model.state.failsafe.phase = FAILSAFE_RX_LOSS_DETECTED;
      for(size_t i = 0; i < _model.state.inputChannelCount; i++)
      {
        setInput((Axis)i, getFailsafeValue(i), true, true);
      }
    }

    void failsafeStage2()
    {
      _model.state.failsafe.phase = FAILSAFE_RX_LOSS_DETECTED;
      if(_model.isActive(MODE_ARMED))
      {
        _model.state.failsafe.phase = FAILSAFE_LANDED;
        _model.disarm();
      }
    }

    void filterInput(InputStatus status)
    {
      Stats::Measure filterMeasure(_model.state.stats, COUNTER_INPUT_FILTER);

      bool newFrame = status != INPUT_IDLE;
      if(newFrame)
      {
        _step = 0.0f;
      }

      const float interpolationStep = _model.state.loopTimer.intervalf * _model.state.inputInterpolationRate;
      if(_step < 1.f) _step += interpolationStep;

      for(size_t c = 0; c < _model.state.inputChannelCount; c++)
      {
        float v = _model.state.inputBuffer[c];
        if(c <= AXIS_THRUST && _model.config.input.interpolationMode != INPUT_INTERPOLATION_OFF)
        {
          float p = _model.state.inputBufferPrevious[c];
          v =_interpolate(p, v, _step);
          v = _model.state.inputFilter[c].update(v);
        }
        setInput((Axis)c, v, newFrame);
      }
    }

    void updateFrameRate(uint32_t now)
    {
      _model.state.inputFrameDelta = (now - _model.state.inputFrameTime);
      _model.state.inputFrameTime = now;
      _model.state.inputFrameRate = 1000000ul / _model.state.inputFrameDelta;

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
      _model.state.inputInterpolationRate = 1.0f / _model.state.inputInterpolationDelta;
    }

    Device::InputDevice * getInputDevice()
    {
      Device::SerialDevice * serial = _model.getSerialStream(SERIAL_FUNCTION_RX_SERIAL);
      if(serial && _model.isActive(FEATURE_RX_SERIAL) && _model.config.input.serialRxProvider == SERIALRX_SBUS)
      {
        _sbus.begin(serial);
        _model.logger.info().logln(F("RX SBUS"));
        return &_sbus;
      }
      else if(_model.isActive(FEATURE_RX_PPM) && _model.config.pin[PIN_INPUT_RX] != -1)
      {
        _ppm.begin(_model.config.pin[PIN_INPUT_RX], _model.config.input.ppmMode);
        _model.logger.info().log(F("RX PPM")).log(_model.config.pin[PIN_INPUT_RX]).logln(_model.config.input.ppmMode);
        return &_ppm;
      }
      return nullptr;
    }

  private:
    float _interpolate(float left, float right, float step)
    {
      return (left * (1.f - step) + right * step);
    }

    Model& _model;
    Device::InputDevice * _device;
    Filter _filter[INPUT_CHANNELS];
    float _step;
    Device::InputPPM _ppm;
    Device::InputSBUS _sbus;

    static const uint32_t TENTH_TO_US = 100000UL;  // 1_000_000 / 10;
    static const uint32_t FRAME_TIME_DEFAULT_US = 23000; // 23 ms
};

}

#endif
