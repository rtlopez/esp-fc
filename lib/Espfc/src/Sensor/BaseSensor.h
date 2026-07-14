#pragma once

#include <helper_3dmath.hpp>
#include <cstdint>

namespace Espfc::Sensor {

class BaseSensor
{
public:
  void align(VectorFloat& dest, uint8_t rotation);
  void toVector(VectorInt16& v, uint8_t * buf);
};

}
