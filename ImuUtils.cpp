#include "ImuUtils.h"

void ImuUtils::hardReset()
{
	q0 = 1.0f;
	q1 = 0.0f; // TODO: set a flag to immediately capture
	q2 = 0.0f; // magnetic orientation on next update
	q3 = 0.0f;
}

void ImuUtils::softReset(float accel[])
{
	// We will use accel/mag readings to reset quats
	calculateRollPitchWithAccel(accel);
	mYaw = 0.0f;
	resetQuat();
}

void ImuUtils::UpdateImu(float *quat)
{
	float q0t = quat[0];
	float q1t = quat[1];
	float q2t = quat[2];
	float q3t = quat[3];
	mRoll = atan2(2.0 * (q0t * q1t + q2t * q3t), (1.0 - 2.0 * (q1t * q1t + q2t * q2t)));
	mPitch = asin(2 * (q0t * q2t - q3t * q1t));
	float new_yaw = atan2(2.0 * (q0t * q3t + q1t * q2t), (1.0 - 2.0 * (q2t * q2t + q3t * q3t)));
	float dYaw = new_yaw - yaw_dmp_;
	yaw_dmp_ = new_yaw;
	mYaw += dYaw;
	if (mYaw < 0)
	{
		mYaw += TWOPI;
	}
	else if (mYaw >= TWOPI)
	{
		mYaw -= TWOPI;
	}
	resetQuat();
}

void ImuUtils::RotateDeltaYaw(float delta_yaw)
{

	mYaw = atan2(2.0 * (q0 * q3 + q1 * q2), (1.0 - 2.0 * (q2 * q2 + q3 * q3)));
	mYaw += delta_yaw;
	resetQuat();
}

void ImuUtils::calculateRollPitchWithAccel(float accel[])
{
	// Calculate roll
	mRoll = 0;
	if (accel[2] != 0)
	{
		mRoll = atan2(-accel[1], accel[2]);
	}
	// Calculate pitch
	mPitch = 0;
	//float denom = aAccel[1] * sin(mRoll) + aAccel[2] * cos(mRoll);
	float denom = sqrt(accel[1] * accel[1] + accel[2] * accel[2]);
	if (denom != 0)
	{
		mPitch = atan(accel[0] / denom);
	}
}

void ImuUtils::resetQuat()
{
	float cr = cos(mRoll * 0.5F);
	float sr = sin(mRoll * 0.5F);
	float cp = cos(mPitch * 0.5F);
	float sp = sin(mPitch * 0.5F);
	float cy = cos(mYaw * 0.5F);
	float sy = sin(mYaw * 0.5F);
	float crcp = cr*cp;
	float spsy = sp*sy;
	float spcy = sp*cy;
	float srcp = sr*cp;

	q0 = crcp * cy + sr * spsy;
	q1 = srcp * cy - cr * spsy;
	q2 = cr * spcy + srcp * sy;
	q3 = crcp * sy - sr * spcy;
}

void ImuUtils::UpdateQuaternions(float roll, float pitch, float yaw)
{
	float sinroll_2 = sin(roll / 2.0F);
	float cosroll_2 = cos(roll / 2.0F);
	float sinpitch_2 = sin(pitch / 2.0F);
	float cospitch_2 = cos(pitch / 2.0F);
	float sinyaw_2 = sin(yaw / 2.0F);
	float cosyaw_2 = cos(yaw / 2.0);
	q0 = cosyaw_2 * cospitch_2 * cosroll_2 + sinyaw_2 * sinpitch_2 * sinroll_2;
	q1 = cosyaw_2 * cospitch_2 * sinroll_2 - sinyaw_2 * sinpitch_2 * cosroll_2;
	q2 = cosyaw_2 * sinpitch_2 * cosroll_2 + sinyaw_2 * cospitch_2 * sinroll_2;
	q3 = sinyaw_2 * cospitch_2 * cosroll_2 - cosyaw_2 * sinpitch_2 * sinroll_2;
}

void ImuUtils::calculateRollPitchYaw()
{
	mRoll = atan2(2.0 * (q0 * q1 + q2 * q3), (1.0 - 2.0 * (q1 * q1 + q2 * q2)));
	mPitch = asin(2 * (q0 * q2 - q3 * q1));
	mYaw = atan2(2.0 * (q0 * q3 + q1 * q2), (1.0 - 2.0 * (q2 * q2 + q3 * q3)));
	if (mYaw < 0)
	{
		mYaw += TWOPI;
	}
	else if (mYaw >= TWOPI)
	{
		mYaw -= TWOPI;
	}
}

void ImuUtils::GetAttitude(float *x)
{
	x[0] = mRoll;
	x[1] = mPitch;
	x[2] = mYaw;
}

float ImuUtils::getRoll()
{
	return mRoll;
}

float ImuUtils::getPitch()
{
	return mPitch;
}

float ImuUtils::getYaw()
{
	float y = mYaw;
	if (y > TWOPI)
	{
		y -= TWOPI;
	}
	else if (y < 0)
	{
		y += TWOPI;
	}
	return (y);
}

void ImuUtils::getAccelBody(float *x)
{
	x[0] = aAccelBody[0];
	x[1] = aAccelBody[1];
	x[2] = aAccelBody[2];
}

void ImuUtils::getGyro(float *x)
{
	x[0] = aGyro[0];
	x[1] = aGyro[1];
	x[2] = aGyro[2];
}

void ImuUtils::getQuat(float *x)
{
	x[0] = q0;
	x[1] = q1;
	x[2] = q2;
	x[3] = q3;
}

//---------------------------------------------------------------------------------------------