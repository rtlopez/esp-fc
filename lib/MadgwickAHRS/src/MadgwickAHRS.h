//=============================================================================================
// MadgwickAHRS.h
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
#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h
#include <math.h>
#include <Arduino.h>

//--------------------------------------------------------------------------------------------
// Variable declaration
class MadgwickAHRS {
public:
  float q0;
  float q1;
  float q2;
  float q3;	// quaternion of sensor frame relative to auxiliary frame
private:
    static float invSqrt(float x) ICACHE_RAM_ATTR;
    float beta;				// algorithm gain
    float invSampleFreq;
    float roll;
    float pitch;
    float yaw;
    char anglesComputed;
    void computeAngles();

//-------------------------------------------------------------------------------------------
// Function declarations
public:
    MadgwickAHRS();
    void begin(float sampleFrequency) { invSampleFreq = 1.0f / sampleFrequency; }
    void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
    void update(float gx, float gy, float gz, float ax, float ay, float az);
    float getRoll() {
        if (!anglesComputed) computeAngles();
        return roll;
    }
    float getPitch() {
        if (!anglesComputed) computeAngles();
        return pitch;
    }
    float getYaw() {
        if (!anglesComputed) computeAngles();
        return yaw;
    }
    void setBeta(float b) {
        beta = b;
    }

};
#endif
