//=============================================================================================
// Madgwick.h
//=============================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// From the x-io website "Open-source resources available on this website are
// provided under the GNU General Public Licence unless an alternative licence
// is provided in source."
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=============================================================================================
#ifndef Madgwick_h
#define Madgwick_h

#include "helper_3dmath.h"

//--------------------------------------------------------------------------------------------
// Variable declaration
class Madgwick {
  private:
    float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame
    float beta;				// algorithm gain
    float invSampleFreq;
    float roll, pitch, yaw;
    bool anglesComputed;

    void computeAngles();

//-------------------------------------------------------------------------------------------
// Function declarations
  public:
    Madgwick();
    void begin(float sampleFrequency) { invSampleFreq = 1.0f / sampleFrequency; }

    void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
    void update(float gx, float gy, float gz, float ax, float ay, float az);

    void setKp(float p) {
      beta = p;
    }
    void setKi(float i) {
      (void)i;
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
