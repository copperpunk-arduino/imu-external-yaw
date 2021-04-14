#ifndef _IMUEXTERNALYAW_H_
#define _IMUEXTERNALYAW_H_
#include <math.h>

class ImuExternalYaw
{
public:
	void resetQuatUpright();
	void resetQuatWithAccel(float accel[]);
	void updateQuaternionsFromEulerRad(float roll_rad, float pitch_rad, float yaw_rad);
	void updateImuWithQuatDifferentialYaw(float q[]);
	void rotateYawRad(float delta_yaw_rad);
	void calculateRollPitchYaw();
	void getAttitudeRad(float attitude[]);
	float getRollRad();
	float getPitchRad();
	float getYawRad();
	void getQuat(float quat[]);

private:
	const float kTwoPi = 6.283185307f;
	float q0_ = 1.0f;
	float q1_ = 0.0f;
	float q2_ = 0.0f;
	float q3_ = 0.0f;
	float yaw_prev_rad_ = 0.0;	
	// Euler
	float roll_rad_;
	float pitch_rad_;
	float yaw_rad_;

	void calculateRollPitchWithAccel(float accel[]);
	void resetQuat();
};

#endif // _IMUEXTERNALYAW_H_
