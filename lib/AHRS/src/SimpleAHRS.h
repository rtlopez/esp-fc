#ifndef _SIMPLE_AHRS_H_
#define _SIMPLE_AHRS_H_

#include "helper_3dmath.h"

class SimpleAHRS
{
  public:
    SimpleAHRS(): _dt(1.f / 100.f), _gain(0.002f), _gravity(0.f, 0.f, 1.f) {}
    
    void begin(float sampleFrequency)
    {
      _dt = 1.0f / sampleFrequency;
    }

    void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
    {
      // http://philstech.blogspot.com/2014/09/fast-quaternion-integration-for.html
      // http://philstech.blogspot.com/2015/06/quaternion-imu-drift-compensation.html
      // http://philstech.blogspot.com/2015/06/complimentary-filter-example-quaternion.html

      VectorFloat g(gx, gy, gz);
      VectorFloat a(ax, ay, az);
      VectorFloat m(mx, my, mz);

      a.normalize();
      a.rotate(_q);
      a.normalize();

      // simple correction
      //VectorFloat err = a.cross(_gravity);
      //err *= _gain * 100.f;
      //err.rotate(_q.getConjugate());
      //g += err;
      
      // more precise correction
      VectorFloat axis = a.cross(_gravity);
      float angle = a.dot(_gravity);
      Quaternion err;
      err.fromAngleVector(angle * _gain, axis);
      err.normalize();
      _q = err * _q;

      // simple prediction
      Quaternion q;
      float halfDt = _dt * 0.5f;
      q.x = g.x * halfDt;
      q.y = g.y * halfDt;
      q.z = g.z * halfDt;
      q.w = 1.f - 0.5 * (q.x * q.x + q.y * q.y + q.z * q.z);
      //q.fromAngularVelocity(g, _dt); // more precise prediction
      q.normalize();
      _q = q * _q;

      _q.normalize();
      _e.eulerFromQuaternion(_q);
    }

	  void updateOld(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
    {
      VectorFloat g(gx, gy, gz);
      VectorFloat a(ax, ay, az);
      VectorFloat m(mx, my, mz);

      // ***** Gyro rotation quaternion *******
      //Quaternion gyro_xy_q;
      //gyro_xy_q.fromAngularVelocity(VectorFloat(g.x, g.y, 0), _dt);
      //gyro_xy_q.normalize();

      //Quaternion gyro_z_q;
      //gyro_z_q.fromAngularVelocity(VectorFloat(0, 0, g.z), _dt);
      //gyro_z_q.normalize();

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
      //_q_xy = _q_xy * gyro_xy_q;
      //_q_xy.normalize(); //make sure q remains a unit quaternion

      //_q_z = _q_z * gyro_z_q;
      //_q_z.normalize(); //make sure q remains a unit quaternion

      //_q = _q * gyro_q;
      //_q.normalize(); //make sure q remains a unit quaternion

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
      //Quaternion acc_xy_q = a.accelToQuaternion();

      // ***** Orientation drift correction *******
      //_q = Quaternion::slerp(_q, acc_q, _gain);
      // simplified with lerp
      //_q_xy = Quaternion::lerp(_q_xy, acc_xy_q, _gain);
      //_q_xy.normalize();

      VectorFloat estimated(0.f, 0.f, 1.f);
      estimated.rotate(_q);
      Quaternion error_q = VectorFloat::diffVectors(estimated, a, _gain);
      //error_q = Quaternion::slerp(Quaternion(), error_q, _gain);
      //Quaternion error_q = VectorFloat::diffVectors(a, estimated, _gain);

      //_q_xy = _q_xy * error_q;
      
      //_q = error_q * _q;
      //_q = gyro_q * _q;

      _q *= error_q;
      _q *= gyro_q;

      //_q = _q_xy;
      //_q = _q_xy * _q_z;
      //_q = acc_xy_q;
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
    Quaternion _q_xy;
    Quaternion _q_z;
    Quaternion _q;
    VectorFloat _e;
    float _dt;
    float _gain;
    const VectorFloat _gravity;
};

#endif