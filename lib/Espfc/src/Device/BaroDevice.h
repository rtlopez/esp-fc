#ifndef _ESPFC_DEVICE_BARO_DEVICE_H_
#define _ESPFC_DEVICE_BARO_DEVICE_H_

#include "BusDevice.h"
#include "BusAwareDevice.h"

namespace Espfc {

enum BaroDeviceType {
  BARO_DEFAULT = 0,
  BARO_NONE    = 1,
  BARO_BMP085  = 2,
  BARO_MS5611  = 3,
  BARO_BMP280  = 4,
  BARO_SPL06   = 5,
  BARO_MAX
};

enum BaroDeviceMode {
  BARO_MODE_TEMP,
  BARO_MODE_PRESS,
};

namespace Device {

class BaroDevice: public BusAwareDevice
{
  public:
    typedef BaroDeviceType DeviceType;

    virtual int begin(BusDevice * bus) = 0;
    virtual int begin(BusDevice * bus, uint8_t addr) = 0;

    virtual DeviceType getType() const = 0;

    virtual float readTemperature() = 0;
    virtual float readPressure() = 0;
    virtual int getDelay(BaroDeviceMode mode) const = 0;
    virtual void setMode(BaroDeviceMode mode) = 0;

    virtual bool testConnection() = 0;

    static const char ** getNames()
    {
      static const char* devChoices[] = { PSTR("AUTO"), PSTR("NONE"), PSTR("BMP085"), PSTR("MS5611"), PSTR("BMP280"), PSTR("SPL06-001"), NULL };
      return devChoices;
    }

    static const char * getName(DeviceType type)
    {
      if(type >= BARO_MAX) return PSTR("?");
      return getNames()[type];
    }
};

}

}

#endif
