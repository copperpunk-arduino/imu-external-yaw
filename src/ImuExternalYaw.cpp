#include "ImuExternalYaw.h"

void ImuExternalYaw::setQuaternions(float quat[]) {
  q0_ = quat[0];
  q1_ = quat[1];
  q2_ = quat[2];
  q3_ = quat[3];
}

void ImuExternalYaw::resetQuatUpright()
{
	q0_ = 1.0f;
	q1_ = 0.0f;
	q2_ = 0.0f; 
	q3_ = 0.0f;
}

void ImuExternalYaw::resetQuatWithAccel(float accel[])
{
	calculateRollPitchWithAccel(accel);
	yaw_rad_ = 0.0f;
	resetQuat();
}

void ImuExternalYaw::updateImuWithQuatDifferentialYaw(float q[])
{
	// The roll and pitch can be taken directly from the quaternions
	roll_rad_ = atan2(2.0 * (q[0] * q[1] + q[2] * q[3]), (1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2])));
	pitch_rad_ = asin(2 * (q[0] * q[2] - q[3] * q[1]));
	// The yaw is calculated based on its change from the last IMU update
	// This differential is added to the current value, which is being driven by a source
	// external from the IMU, e.g., a dual-GPS heading measurement
	float new_yaw_rad = atan2(2.0 * (q[0] * q[3] + q[1] * q[2]), (1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3])));
	float d_yaw = new_yaw_rad - yaw_prev_rad_;
	yaw_prev_rad_ = new_yaw_rad;
	yaw_rad_ += d_yaw;
	if (yaw_rad_ < 0)
	{
		yaw_rad_ += kTwoPi;
	}
	else if (yaw_rad_ >= kTwoPi)
	{
		yaw_rad_ -= kTwoPi;
	}
	resetQuat();
}

void ImuExternalYaw::rotateYawRad(float delta_yaw_rad)
{

	yaw_rad_ = atan2(2.0 * (q0_ * q3_ + q1_ * q2_), (1.0 - 2.0 * (q2_ * q2_ + q3_ * q3_)));
	yaw_rad_ += delta_yaw_rad;
	resetQuat();
}

void ImuExternalYaw::calculateRollPitchWithAccel(float accel[])
{
	roll_rad_ = 0;
	if (accel[2] != 0)
	{
		roll_rad_ = atan2(-accel[1], accel[2]);
	}

	pitch_rad_ = 0;
	float denom = sqrt(accel[1] * accel[1] + accel[2] * accel[2]);
	if (denom != 0)
	{
		pitch_rad_ = atan(accel[0] / denom);
	}
}

void ImuExternalYaw::resetQuat()
{
	float cr = cos(roll_rad_ * 0.5F);
	float sr = sin(roll_rad_ * 0.5F);
	float cp = cos(pitch_rad_ * 0.5F);
	float sp = sin(pitch_rad_ * 0.5F);
	float cy = cos(yaw_rad_ * 0.5F);
	float sy = sin(yaw_rad_ * 0.5F);
	float crcp = cr*cp;
	float spsy = sp*sy;
	float spcy = sp*cy;
	float srcp = sr*cp;

	q0_ = crcp * cy + sr * spsy;
	q1_ = srcp * cy - cr * spsy;
	q2_ = cr * spcy + srcp * sy;
	q3_ = crcp * sy - sr * spcy;
}

void ImuExternalYaw::updateQuaternionsFromEulerRad(float roll_rad, float pitch_rad, float yaw_rad)
{
	float cr = cos(roll_rad * 0.5F);
	float sr = sin(roll_rad_ * 0.5F);
	float cp = cos(pitch_rad * 0.5F);
	float sp = sin(pitch_rad * 0.5F);
	float cy = cos(yaw_rad * 0.5F);
	float sy = sin(yaw_rad * 0.5F);
	float crcp = cr*cp;
	float spsy = sp*sy;
	float spcy = sp*cy;
	float srcp = sr*cp;

	q0_ = crcp * cy + sr * spsy;
	q1_ = srcp * cy - cr * spsy;
	q2_ = cr * spcy + srcp * sy;
	q3_ = crcp * sy - sr * spcy;
}

void ImuExternalYaw::calculateRollPitchYaw()
{
	roll_rad_ = atan2(2.0 * (q0_ * q1_ + q2_ * q3_), (1.0 - 2.0 * (q1_ * q1_ + q2_ * q2_)));
	pitch_rad_ = asin(2 * (q0_ * q2_ - q3_ * q1_));
	yaw_rad_ = atan2(2.0 * (q0_ * q3_ + q1_ * q2_), (1.0 - 2.0 * (q2_ * q2_ + q3_ * q3_)));
	if (yaw_rad_ < 0)
	{
		yaw_rad_ += kTwoPi;
	}
	else if (yaw_rad_ >= kTwoPi)
	{
		yaw_rad_ -= kTwoPi;
	}
}

void ImuExternalYaw::getAttitudeRad(float attitude[])
{
	attitude[0] = roll_rad_;
	attitude[1] = pitch_rad_;
	attitude[2] = yaw_rad_;
}

float ImuExternalYaw::getRollRad()
{
	return roll_rad_;
}

float ImuExternalYaw::getPitchRad()
{
	return pitch_rad_;
}

float ImuExternalYaw::getYawRad()
{
	float y = yaw_rad_;
	if (y > kTwoPi)
	{
		y -= kTwoPi;
	}
	else if (y < 0)
	{
		y += kTwoPi;
	}
	return (y);
}

void ImuExternalYaw::getQuat(float quat[])
{
	quat[0] = q0_;
	quat[1] = q1_;
	quat[2] = q2_;
	quat[3] = q3_;
}