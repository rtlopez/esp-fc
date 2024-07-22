#ifndef _ESPFC_DEVICE_MAG_DEVICE_H_
#define _ESPFC_DEVICE_MAG_DEVICE_H_

#include "helper_3dmath.h"
#include "BusAwareDevice.h"

namespace Espfc {

enum MagDeviceType {
  MAG_DEFAULT = 0,
  MAG_NONE    = 1,
  MAG_HMC5883 = 2,
  MAG_AK8975  = 3,
  MAG_AK8963  = 4,
  MAG_QMC5883 = 5,
  MAG_MAX
};

namespace Device {

class MagDevice: public BusAwareDevice
{
  public:
    typedef MagDeviceType DeviceType;

    virtual int begin(BusDevice * bus) = 0;
    virtual int begin(BusDevice * bus, uint8_t addr) = 0;

    virtual DeviceType getType() const = 0;

    virtual int readMag(VectorInt16& v) = 0;
    virtual const VectorFloat convert(const VectorInt16& v) const = 0;
    virtual int getRate() const = 0;

    virtual bool testConnection() = 0;

    static const char ** getNames()
    {
      static const char* devChoices[] = { PSTR("AUTO"), PSTR("NONE"), PSTR("HMC5883L"), PSTR("AK8975"), PSTR("AK8963"), PSTR("QMC5883L"),NULL };
      return devChoices;
    }

    static const char * getName(DeviceType type)
    {
      if(type >= MAG_MAX) return PSTR("?");
      return getNames()[type];
    }
};

}

}

#endif
