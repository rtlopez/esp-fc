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
    int setPitMode();
    int trampSendCommand(uint8_t cmd, uint16_t param);
    Connect::VtxDeviceType type = Connect::VTXDEV_TRAMP;
    const uint16_t TrampFreqTable[9][8] = {
      {5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725}, // BOSCAM_A
      {5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866}, // BOSCAM_B
      {5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945}, // BOSCAM_E
      {5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880}, // FATSHARK
      {5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917}, // RACEBAND
      {5325, 5348, 5366, 5384, 5402, 5420, 5438, 5456}, // U
      {5474, 5492, 5510, 5528, 5546, 5564, 5582, 5600}, // O
      {5333, 5373, 5413, 5453, 5493, 5533, 5573, 5613}, // L
      {5653, 5693, 5733, 5773, 5813, 5853, 5893, 5933}  // H
    };
    const uint16_t TrampPowerTable[5] = {
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
