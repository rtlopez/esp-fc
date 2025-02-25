#pragma once

#include "Device/SerialDevice.h"
#include "Utils/Timer.h"
#include "Model.h"

namespace Espfc::Connect {

enum VtxDeviceType {
  VTXDEV_UNSUPPORTED = 0, // reserved for MSP
  VTXDEV_RTC6705     = 1,
  // 2 reserved
  VTXDEV_SMARTAUDIO  = 3,
  VTXDEV_TRAMP       = 4,
  VTXDEV_MSP         = 5,
  VTXDEV_UNKNOWN     = 0xFF,
};

enum State {
  INACTIVE,
  INIT,
  SET_POWER,
  SET_CHANNEL,
  IDLE,
};

class Vtx
{
  public:
    Vtx(Model& model): _serial(NULL), _model(model) {}

    int begin(Device::SerialDevice * serial);
    int update();
    int setChannel();
    int setPower();
    Connect::VtxDeviceType type;

  private:
    Device::SerialDevice* _serial;
    Model& _model;
    State _state = State::INACTIVE;
    bool _armed = false;
    Utils::Timer _timer;
};

}
