#ifndef _SIMPLE_AHRS_H_
#define _SIMPLE_AHRS_H_

#include "helper_3dmath.h"

class SimpleAHRS
{
  public:
    SimpleAHRS(): _dt(1.f / 100.f), _gain(0.002f) {}
    
    void begin(float sampleFrequency)
    {
      _dt = 1.0f / sampleFrequency;
    }

	  void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
    {
      VectorFloat g(gx, gy, gz);
      VectorFloat a(ax, ay, az);
      VectorFloat m(mx, my, mz);

      // ***** Gyro rotation quaternion *******
      Quaternion gyro_q;
      gyro_q.fromAngularVelocity(g, _dt);
      gyro_q.normalize();

      // other version
      //VectorFloat gyro_e = g * _dt;
      //Quaternion gyro_q = gyro_e.eulerToQuaternion();

      // simplified
      //Quaternion gyro_q(1.f, g.x * _dt * 0.5f, g.y * _dt * 0.5f, g.z * _dt * 0.5f);
      //gyro_q.normalize();

      // ***** Orientation prediction from gyro *******
      // integrate gyro angular displacement 'gyro_q' to orientation 'q'
      _q = _q * gyro_q;
      _q.normalize(); //make sure q remains a unit quaternion

      // ***** Accel orientation quaternion *******
      // by rotation matrix version 
      //VectorFloat east = a.cross(m);
      //VectorFloat north = east.cross(a);
      //Matrix<3,3> rotationMatrix;
      //rotationMatrix.vector_to_col(north, 0);
      //rotationMatrix.vector_to_col(east, 1);
      //rotationMatrix.vector_to_col(a, 2);
      //Quaternion acc_q;
      //acc_q.fromMatrix(rotationMatrix);
      
      // simplified method doesn’t account for magnetic dip
      //VectorFloat acc_e;
      //acc_e.x = atan2f(m.y(), m.x());
      //acc_e.y = atan2f(-a.x, sqrt(a.y * a.y + a.z * a.z));
      //acc_e.z = atan2f(a.y, a.z);
      //Quaternion acc_q = acc_e.eulerToQuaternion();

      // another version, no mag
      Quaternion acc_q = a.accelToQuaternion();

      // yet another version, no mag
      //VectorFloat acc_e = a.accelToEuler();
      //acc_e.z = _e.z;
      //Quaternion acc_q = acc_e.eulerToQuaternion();

      // ***** Orientation drift correction *******
      _q = Quaternion::slerp(_q, acc_q, _gain);
      // simplified with lerp
     // _q = Quaternion::lerp(_q, acc_q, _gain);
      _q.normalize();
      _e.eulerFromQuaternion(_q);
    }

	  void update(float gx, float gy, float gz, float ax, float ay, float az)
    {
      return update(gx, gy, gz, ax, ay, az, 0.f, 0.f, 0.f);
    }

    void setKp(float p) {
		  _gain = p;
	  }

    const Quaternion getQuaternion() const {
		  return _q;
    }
	  const VectorFloat getEuler() const {
      return _e;
	  }

  private:
    Quaternion _q;
    VectorFloat _e;
    float _dt;
    float _gain;
};

#endif