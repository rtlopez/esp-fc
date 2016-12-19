#ifndef _ESPFC_FUSION_H_
#define _ESPFC_FUSION_H_

#include "Model.h"

namespace Espfc {

class Fusion
{
  public:
    Fusion(Model& model): _model(model) {}
    int begin() {}
    int update()
    {

    }

  private:
    Model& _model;

};

}

#endif
