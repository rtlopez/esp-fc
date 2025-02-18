#pragma once

#include "Model.h"
#if defined(ESPFC_I2C_0)
#include "Device/BusI2C.h"
#endif
#if defined(ESPFC_SPI_0)
#include "Device/BusSPI.h"
#endif
#include "Device/BusSlave.h"

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
      _model.logger.info().log(F("SPI")).log(FPSTR(Dev::getName(type))).logln(status ? "Y" : "");
      return status;
    }
#endif

#if defined(ESPFC_I2C_0)
    template<typename Dev>
    bool detectDevice(Dev& dev, Device::BusI2C& bus)
    {
      typename Dev::DeviceType type = dev.getType();
      bool status = dev.begin(&bus);
      _model.logger.info().log(F("I2C")).log(FPSTR(Dev::getName(type))).logln(status ? "Y" : "");
      return status;
    }
#endif

    template<typename Dev>
    bool detectDevice(Dev& dev, Device::BusSlave& bus)
    {
      typename Dev::DeviceType type = dev.getType();
      bool status = dev.begin(&bus);
      _model.logger.info().log(F("SLV")).log(FPSTR(Dev::getName(type))).logln(status ? "Y" : "");
      return status;
    }

    static void restart(const Model& model);

  private:
    Model& _model;
};

}
