#ifndef _ESPFC_MSP_PARSER_H_
#define _ESPFC_MSP_PARSER_H_

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

#endif
