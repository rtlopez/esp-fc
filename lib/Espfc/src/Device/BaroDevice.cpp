#include "Device/BaroDevice.h"
#include "Hal/Pgm.h"
#include <cstddef>

namespace Espfc::Device {

const char ** BaroDevice::getNames()
{
    static const char* devChoices[] = { PSTR("AUTO"), PSTR("NONE"), PSTR("BMP085"), PSTR("MS5611"), PSTR("BMP280"), PSTR("SPL06-001"), NULL };
    return devChoices;
}

const char * BaroDevice::getName(DeviceType type)
{
    if(type >= BARO_MAX) return PSTR("?");
    return getNames()[type];
}

}
