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
      _model.state.pose = poseFromAccelMag(_model.state.accel, _model.state.mag);
    }

    VectorFloat poseFromAccelMag(const VectorFloat& accel, const VectorFloat& mag)
    {
      VectorFloat r = accel.accelToEuler();

      //Quaternionq = r.eulerToQuaternion();
      //VectorFloat m = mag.getRotated(q);

      // Quaternion q = r.eulerToQuaternion();
      // since result.z() is always 0, this can be optimized a little
      float cosX2 = cos(r.x / 2.0f);
      float sinX2 = sin(r.x / 2.0f);
      float cosY2 = cos(r.y / 2.0f);
      float sinY2 = sin(r.y / 2.0f);

      Quaternion q;
      q.w = cosX2 * cosY2;
      q.x = sinX2 * cosY2;
      q.y = cosX2 * sinY2;
      q.z = -sinX2 * sinY2;
      //q.normalize();

      Quaternion m;
      m.w = 0;
      m.x = mag.x;
      m.y = mag.y;
      m.z = mag.z;

      m = q * m * q.getConjugate();

      _model.state.magAccel.x = m.x;
      _model.state.magAccel.y = m.y;
      _model.state.magAccel.z = m.z;

      r.z = -atan2(m.y, m.x);

      return r;
    }

  private:
    Model& _model;

};

}

#endif
