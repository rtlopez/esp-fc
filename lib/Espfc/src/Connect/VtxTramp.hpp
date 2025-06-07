#pragma once

#include "Device/SerialDevice.h"
#include "Utils/Timer.h"
#include "Model.h"
#include "Vtx.h"

namespace Espfc::Connect {

class VtxTramp
{
  public:
    VtxTramp(Model& model): _serial(NULL), _model(model) {}

    int begin(Device::SerialDevice * serial);
    int initTramp();
    int update();
    int setChannel();
    int setPower();
    int trampSendCommand(uint8_t cmd, uint16_t param);
    Connect::VtxDeviceType type = Connect::VTXDEV_TRAMP;
    const uint16_t TrampFreqTable[8][8] {
      {5865, 5845, 5825, 5805, 5785, 5765, 5745, 0},
      {5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866},
      {5740, 5760, 5780, 5800, 5820, 5840, 5860, 0},
      {0, 0, 5732, 5769, 5806, 5843, 0, 0},
      {5732, 5765, 5828, 5840, 5866, 5740, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0},
    };
    const uint16_t TrampPowerTable[5] {
      25, 100, 200, 400, 600
    };


  private:
    Device::SerialDevice* _serial;
    Model& _model;
    State _state = State::INACTIVE;
    bool _armed = false;
    Utils::Timer _timer;
};

}
