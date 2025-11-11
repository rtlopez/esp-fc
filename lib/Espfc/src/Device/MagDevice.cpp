#include "Device/MagDevice.h"
#include "Hal/Pgm.h"
#include <cstddef>

namespace Espfc::Device {

const char ** MagDevice::getNames()
{
    static const char* devChoices[] = { PSTR("AUTO"), PSTR("NONE"), PSTR("HMC5883L"), PSTR("AK8975"), PSTR("AK8963"), PSTR("QMC5883L"),PSTR("QMC5883P"),NULL };
    return devChoices;
}

const char * MagDevice::getName(DeviceType type)
{
    if(type >= MAG_MAX) return PSTR("?");
    return getNames()[type];
}

}
