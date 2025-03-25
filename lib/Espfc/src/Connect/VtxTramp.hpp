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
    uint8_t header[2];
    uint8_t command;
    uint8_t payload[12];
    uint8_t crc;
    uint8_t terminator;

    TrampCommand() 
        : header{0x0F, 0x00}, command(0), payload{0}, crc(0), terminator(0x00) 
    {
    }
};


}
