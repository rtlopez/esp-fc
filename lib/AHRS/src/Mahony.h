//=============================================================================================
// Mahony.h
//=============================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=============================================================================================
#ifndef Mahony_h
#define Mahony_h

#include "helper_3dmath.h"

//--------------------------------------------------------------------------------------------
// Variable declaration

class Mahony {
private:
  float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame
	float twoKp;		      // 2 * proportional gain (Kp)
	float twoKi;		      // 2 * integral gain (Ki)
	float integralFBx, integralFBy, integralFBz;  // integral error terms scaled by Ki
	float invSampleFreq;
	float roll, pitch, yaw;
	bool anglesComputed;

	void computeAngles();

//-------------------------------------------------------------------------------------------
// Function declarations

public:
	Mahony();
	void begin(float sampleFrequency) { invSampleFreq = 1.0f / sampleFrequency; }

	void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
	void update(float gx, float gy, float gz, float ax, float ay, float az);
	
	void setKp(float p) {
		twoKp = p;
	}
	void setKi(float i) {
		twoKi = i;
	}
	const Quaternion getQuaternion() const {
		return Quaternion(q0, q1, q2, q3);
  }
	const VectorFloat getEuler() {
		if (!anglesComputed) computeAngles();
		return VectorFloat(roll, pitch, yaw);
	}
};

#endif
