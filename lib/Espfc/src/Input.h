#pragma once

#include "Device/Input/InputCRSF.hpp"
#include "Device/Input/InputIBUS.hpp"
#include "Device/Input/InputPPM.hpp"
#include "Device/Input/InputSBUS.hpp"
#include "Device/InputDevice.hpp"
#include "Model.h"
#include "TelemetryManager.h"
#if defined(ESPFC_ESPNOW)
#include "Device/Input/InputEspNow.hpp"
#endif

namespace Espfc {

enum FailsafeChannelMode
{
  FAILSAFE_MODE_AUTO,
  FAILSAFE_MODE_HOLD,
  FAILSAFE_MODE_SET,
  FAILSAFE_MODE_INVALID
};

enum InputPwmRange
{
  PWM_RANGE_MIN = 1000,
  PWM_RANGE_MID = 1500,
  PWM_RANGE_MAX = 2000
};

class Input
{
public:
  Input(Model& model, TelemetryManager& telemetry);

  int begin();
  int reload(ModelChangeEvent event);
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
  Device::InputDevice* getInputDevice();

private:
  Model& _model;
  TelemetryManager& _telemetry;
  Device::InputDevice* _device;
  Utils::Filter _filter[INPUT_CHANNELS];
  Device::Input::InputPPM _ppm;
  Device::Input::InputIBUS _ibus;
  Device::Input::InputSBUS _sbus;
  Device::Input::InputCRSF _crsf;
#if defined(ESPFC_ESPNOW)
  Device::Input::InputEspNow _espnow;
#endif

  static constexpr uint32_t TENTH_TO_US = 100000UL;        // 1_000_000 / 10;
  static constexpr uint32_t FRAME_TIME_DEFAULT_US = 23000; // 23 ms
};

} // namespace Espfc
