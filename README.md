# ImuExternalYaw
Helper functions for integrating IMU with external heading measurements. It may have some functions that are useful for generic IMU manipulation.

A simple example will be coming soon. For now, please look to the [IcmInsDualGps](https://github.com/copperpunk-arduino/icm-ins-dual-gps) repository to see how this library is used.<br>
The key functions are as follows:<br>
*   `updateImuWithQuatDifferentialYaw(float [])` - takes the quaternions published by the ICM-20948 GAME_ROTATION_VECTOR, which does not utilize the magnetometer for yaw measurements.
*   `rotateYawRad(float)` - uses the correction provided by the dual-GPS Extended Kalman Filter algorithm to update the yaw to its absolute value. NOTE: this value is the change in absolute yaw from the last update. The ICM-20948 yaw measurement cannot be synced, and thus we must keep track of the differentials instead of the absolutes. This seems like the only way, but if there's a better one, please make a suggestion.
