#pragma once

#include "Connect/Msp.hpp"

namespace Espfc {

namespace Connect {

class MspParser
{
  public:
    MspParser();
    void parse(char c, MspMessage& msg);
};

}

}
