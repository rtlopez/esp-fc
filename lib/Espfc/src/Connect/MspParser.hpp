#pragma once

#include "Connect/Msp.hpp"

namespace Espfc::Connect {

class MspParser
{
public:
  MspParser();
  void parse(char c, MspMessage& msg);
};

} // namespace Espfc::Connect
