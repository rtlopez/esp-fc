#include "Device/BusDevice.h"
#include "Hal/Pgm.h"
#include <cstddef>

namespace Espfc::Device {

const char ** BusDevice::getNames()
{
    static const char* busDevChoices[] = { PSTR("NONE"), PSTR("AUTO"), PSTR("I2C"), PSTR("SPI"), PSTR("SLV"), NULL };
    return busDevChoices;
}

const char * BusDevice::getName(BusType type)
{
    if(type >= BUS_MAX) return PSTR("?");
    return getNames()[type];
}

}
