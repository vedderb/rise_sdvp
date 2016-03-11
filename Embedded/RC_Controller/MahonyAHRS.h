//=====================================================================================================
// MahonyAHRS.h
//=====================================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#ifndef MahonyAHRS_h
#define MahonyAHRS_h

#include "conf_general.h"

// Function declarations
void MahonyAHRSInitAttitudeInfo(ATTITUDE_INFO *att);
void MahonyAHRSupdateInitialOrientation(float *accelXYZ, float *magXYZ, ATTITUDE_INFO *att);
void MahonyAHRSupdate(float *gyroXYZ, float *accelXYZ, float *magXYZ, float dt, ATTITUDE_INFO *att);
void MahonyAHRSupdateIMU(float *gyroXYZ, float *accelXYZ, float dt, ATTITUDE_INFO *att);
float MahonyAHRSGetRoll(ATTITUDE_INFO *att);
float MahonyAHRSGetPitch(ATTITUDE_INFO *att);
float MahonyAHRSGetYaw(ATTITUDE_INFO *att);
void MahonyAHRSGetRollPitchYaw(float *rpy, ATTITUDE_INFO *att);

#endif
