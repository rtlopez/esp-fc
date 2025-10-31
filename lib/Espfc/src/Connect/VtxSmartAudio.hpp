#pragma once

#include "Device/SerialDevice.h"
#include "Utils/Timer.h"
#include "Model.h"
#include "Vtx.h"

namespace Espfc::Connect {

class VtxSmartAudio
{
  public:
    VtxSmartAudio(Model& model): _serial(NULL), _model(model) {}

    int begin(Device::SerialDevice * serial);
    int update();
    int setChannel();
    int setPower();
    Connect::VtxDeviceType type = Connect::VTXDEV_SMARTAUDIO;

  private:
    Device::SerialDevice* _serial;
    Model& _model;
    State _state = State::INACTIVE;
    bool _armed = false;
    Utils::Timer _timer;
};

}
