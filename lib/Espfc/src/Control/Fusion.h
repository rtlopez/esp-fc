#pragma once

#include "Model.h"
#include <Madgwick.hpp>
#include <Mahony.hpp>
#include <Rtqf.hpp>
#include "Utils/Filter.h"

namespace Espfc {

namespace Control {

class Fusion
{
  public:
    Fusion(Model& model);
    int begin();
    void restoreGain();
    int update();

  private:
    Quaternion madgwickFusion(VectorFloat g, VectorFloat a, VectorFloat m);
    Quaternion mahonyFusion(VectorFloat g, VectorFloat a, VectorFloat m);
    Quaternion rtqfFusion(VectorFloat g, VectorFloat a, VectorFloat m);
    Quaternion filterQuaternion(const Quaternion& q);

    Model& _model;
    Madgwick _madgwick;
    Mahony _mahony;
    Rtqf _rtqf;
    Utils::Filter _qFilter[4];
    bool _useMag;
};

}

}
