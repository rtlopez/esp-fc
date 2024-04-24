#pragma once

#include <cstdint>
#include "BusDevice.h"
#include "BusAwareDevice.h"

namespace Espfc {

namespace Device {

class BusSlave: public BusDevice, public BusAwareDevice
{
  public:
    BusSlave();

    int begin(BusDevice * dev, int addr);

    BusType getType() const override;

    virtual int8_t read(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data) override;

    // readFast() ignores devAddr and regAddr args and read ext sensor data reg from master
    virtual int8_t readFast(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data) override;

    // writes only one byte, length is ignored
    virtual bool write(uint8_t devAddr, uint8_t regAddr, uint8_t length, const uint8_t* data) override;

    int8_t writeMaster(uint8_t regAddr, uint8_t data);

    int8_t readMaster(uint8_t regAddr, uint8_t length, uint8_t *data);
};

}

}
