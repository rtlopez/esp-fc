#pragma once

#include "Model.h"
#include "Utils/Math.hpp"
#include "Device/InputDevice.h"
#include "Device/InputPPM.h"
#include "Device/InputIBUS.hpp"
#include "Device/InputSBUS.h"
#include "Device/InputCRSF.h"
#include "TelemetryManager.h"
#if defined(ESPFC_ESPNOW)
#include "Device/InputEspNow.h"
#endif

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
    Input(Model& model, TelemetryManager& telemetry);

    int begin();
    int update();

    int16_t getFailsafeValue(uint8_t c);
    void setInput(Axis i, float v, bool newFrame, bool noFilter = false);

    InputStatus readInputs();
    void processInputs();

    bool failsafe(InputStatus status);
    void failsafeIdle();
    void failsafeStage1();
    void failsafeStage2();
    void filterInputs(InputStatus status);

    void updateFrameRate();
    Device::InputDevice * getInputDevice();

  private:
    inline float _interpolate(float left, float right, float step)
    {
      return (left * (1.f - step) + right * step);
    }

    Model& _model;
    TelemetryManager& _telemetry;
    Device::InputDevice * _device;
    Utils::Filter _filter[INPUT_CHANNELS];
    float _step;
    Device::InputPPM _ppm;
    Device::InputIBUS _ibus;
    Device::InputSBUS _sbus;
    Device::InputCRSF _crsf;
#if defined(ESPFC_ESPNOW)
    Device::InputEspNow _espnow;
#endif

    static constexpr uint32_t TENTH_TO_US = 100000UL;  // 1_000_000 / 10;
    static constexpr uint32_t FRAME_TIME_DEFAULT_US = 23000; // 23 ms
};

}
