#include "Device/BusDevice.h"
#include "Hal/Pgm.h"
#include <cstddef>

namespace Espfc::Device {

const char ** BusDevice::getNames()
{
    static const char* devChoices[] = { PSTR("AUTO"), PSTR("NONE"), PSTR("HMC5883L"), PSTR("AK8975"), PSTR("AK8963"), PSTR("QMC5883L"), PSTR("QMC5883P"),NULL };
    return busDevChoices;
}

const char * BusDevice::getName(BusType type)
{
    if(type >= BUS_MAX) return PSTR("?");
    return getNames()[type];
}

}
