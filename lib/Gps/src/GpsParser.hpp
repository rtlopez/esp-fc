#pragma once

#include "GpsProtocol.hpp"
#include <cstdint>

namespace Gps {

class UbxParser
{
public:
  void parse(uint8_t c, UbxMessage& m);
};

class NmeaParser
{
public:
  void parse(uint8_t c, NmeaMessage& m);
};

} // namespace Gps
