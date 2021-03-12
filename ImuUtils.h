#ifndef _IMUUTILS_H_
#define _IMUUTILS_H_

#include <InsConstants.h>
#include "Arduino.h"
class ImuUtils
{
public:
	// ImuUtils();
	void hardReset();
	void softReset(float accel[]);
	void UpdateQuaternions(float roll, float pitch, float yaw);
	void UpdateImu(float *quat); // 10x1: (3x1 accel,3x1 gyro,3x1 mag, mag flag)
	void RotateDeltaYaw(float delta_yaw);
	void calculateRollPitchYaw();
	void GetAttitude(float *x);
	float getRoll();
	float getPitch();
	float getYaw();
	void getAccelBody(float *x);
	void getGyro(float *x);
	void getQuat(float *x);

private:
	float q0 = 1.0f;
	float q1 = 0.0f;
	float q2 = 0.0f;
	float q3 = 0.0f;
	float yaw_dmp_ = 0.0;	
	// Accel
	float aAccelBody[3];
	// Gyro
	float aGyro[3]; // slow (typically 25Hz) averaged readings (g)
	// Euler
	float mRoll, mPitch, mYaw;

	void calculateRollPitchWithAccel(float accel[]);
	void resetQuat();
};

#endif // _IMUUTILS_H_
