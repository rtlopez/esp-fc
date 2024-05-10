#pragma once

#include "Model.h"
#include "Device/SerialDevice.h"
#include "BlackboxSerialBuffer.h"
extern "C" {
#include <platform.h>
}

namespace Espfc {

namespace Blackbox {

class Blackbox
{
  public:
    Blackbox(Model& model);
    int begin();
    int update();

  private:
    void updateData();
    void updateArmed();
    void updateMode();

    Model& _model;
    pidProfile_s _pidProfile;
    Device::SerialDevice * _serial;
    BlackboxSerialBuffer _buffer;
};

}

}
