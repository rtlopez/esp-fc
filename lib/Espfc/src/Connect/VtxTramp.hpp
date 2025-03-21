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
    Connect::VtxDeviceType type = Connect::VTXDEV_TRAMP;

  private:
    Device::SerialDevice* _serial;
    Model& _model;
    State _state = State::INACTIVE;
    bool _armed = false;
    Utils::Timer _timer;
};


struct TrampCommand {
    uint8_t header[2] = {0x0F, 0x00}; // Header bytes
    uint8_t command;                  // Command identifier
    uint8_t payload[12] = {0};        // Data payload
    uint8_t crc;                      // CRC byte
    uint8_t terminator = 0x00;        // Terminator byte
};


}
