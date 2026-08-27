#pragma once

#include "Model.h"
#if defined(ESPFC_I2C_0)
#include "Device/BusI2C.hpp"
#endif
#if defined(ESPFC_SPI_0)
#include "Device/BusSPI.hpp"
#endif
#include "Device/BusSlave.hpp"

namespace Espfc {

class Hardware
{
public:
  Hardware(Model& model);
  int begin();
  void onI2CError();

  void initBus();
  void detectGyro();
  void detectMag();
  void detectBaro();

#if defined(ESPFC_SPI_0)
  template<typename Dev>
  bool detectDevice(Dev& dev, Device::BusSPI& bus, int cs)
  {
    typename Dev::DeviceType type = dev.getType();
    bool status = dev.begin(&bus, cs);
    auto& logger = _model.logger.info();
    logger.log("SPI").log(Dev::getName(type)).loghex(dev.getChipId().value_or(0xff)).logln(status ? "Y" : "");
    return status;
  }
#endif

#if defined(ESPFC_I2C_0)
  template<typename Dev>
  bool detectDevice(Dev& dev, Device::BusI2C& bus)
  {
    typename Dev::DeviceType type = dev.getType();
    bool status = dev.begin(&bus);
    auto& logger = _model.logger.info();
    logger.log("I2C").log(Dev::getName(type)).loghex(dev.getChipId().value_or(0xff)).logln(status ? "Y" : "");
    return status;
  }
#endif

  template<typename Dev>
  bool detectDevice(Dev& dev, Device::BusSlave& bus)
  {
    typename Dev::DeviceType type = dev.getType();
    bool status = dev.begin(&bus);
    auto& logger = _model.logger.info();
    logger.log("SLV").log(Dev::getName(type)).loghex(dev.getChipId().value_or(0xff)).logln(status ? "Y" : "");
    return status;
  }

  static void restart(const Model& model);

private:
  Model& _model;
};

} // namespace Espfc
