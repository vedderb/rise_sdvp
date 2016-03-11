//=====================================================================================================
// MahonyAHRS.c
//=====================================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
// 26/01/2014	Benjamin V		Adaption to our platform
//
//=====================================================================================================

// Header files
#include "MahonyAHRS.h"
#include "utils.h"
#include <math.h>

// Settings
#define TWO_KP					(2.0f * 0.3f)	// 2 * proportional gain
#define TWO_KI					(2.0f * 0.0f)	// 2 * integral gain
#define ACC_CONFIDENCE_DECAY	(1.0)

// Function declarations
static float invSqrt(float x);
static float calculateAccConfidence(float accMag, float *accMagP);

// AHRS algorithm initialization
static float calculateAccConfidence(float accMag, float *accMagP) {
	// G.K. Egan (C) computes confidence in accelerometers when
	// aircraft is being accelerated over and above that due to gravity

	float confidence;

	accMag = *accMagP * 0.9f + accMag * 0.1f;
	*accMagP = accMag;

	confidence = 1.0 - (ACC_CONFIDENCE_DECAY * sqrtf(fabsf(accMag - 1.0f)));
	utils_truncate_number(&confidence, 0.0, 1.0);

	return confidence;
}

void MahonyAHRSInitAttitudeInfo(ATTITUDE_INFO *att) {
	att->q0 = 1.0;
	att->q1 = 0.0;
	att->q2 = 0.0;
	att->q3 = 0.0;
	att->integralFBx = 0.0;
	att->integralFBy = 0.0;
	att->integralFBz = 0.0;
	att->accMagP = 1.0;
	att->initialUpdateDone = 0;
}

void MahonyAHRSupdateInitialOrientation(float *accelXYZ, float *magXYZ, ATTITUDE_INFO *att) {
	float initialRoll, initialPitch;
	float cosRoll, sinRoll, cosPitch, sinPitch;
	float magX, magY;
	float initialHdg, cosHeading, sinHeading;

	initialRoll = atan2f(-accelXYZ[1], -accelXYZ[2]);
	initialPitch = atan2f(accelXYZ[0], -accelXYZ[2]);

	cosRoll = cosf(initialRoll);
	sinRoll = sinf(initialRoll);
	cosPitch = cosf(initialPitch);
	sinPitch = sinf(initialPitch);

	magX = magXYZ[0] * cosPitch + magXYZ[1] * sinRoll * sinPitch + magXYZ[2] * cosRoll * sinPitch;
	magY = magXYZ[1] * cosRoll - magXYZ[2] * sinRoll;

	initialHdg = atan2f(-magY, magX);

	cosRoll = cosf(initialRoll * 0.5f);
	sinRoll = sinf(initialRoll * 0.5f);

	cosPitch = cosf(initialPitch * 0.5f);
	sinPitch = sinf(initialPitch * 0.5f);

	cosHeading = cosf(initialHdg * 0.5f);
	sinHeading = sinf(initialHdg * 0.5f);

	att->q0 = cosRoll * cosPitch * cosHeading + sinRoll * sinPitch * sinHeading;
	att->q1 = sinRoll * cosPitch * cosHeading - cosRoll * sinPitch * sinHeading;
	att->q2 = cosRoll * sinPitch * cosHeading + sinRoll * cosPitch * sinHeading;
	att->q3 = cosRoll * cosPitch * sinHeading - sinRoll * sinPitch * cosHeading;
}

/*
 * Functions
 */

// AHRS algorithm update
void MahonyAHRSupdate(float *gyroXYZ, float *accelXYZ, float *magXYZ, float dt, ATTITUDE_INFO *att) {
	float accelNorm, recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;  
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	float gx = gyroXYZ[0];
	float gy = gyroXYZ[1];
	float gz = gyroXYZ[2];

	float ax = accelXYZ[0];
	float ay = accelXYZ[1];
	float az = accelXYZ[2];

	float mx = magXYZ[0];
	float my = magXYZ[1];
	float mz = magXYZ[2];

	volatile float twoKp = TWO_KP;
	volatile float twoKi = TWO_KI;
	float accelConfidence;

	if (!att->initialUpdateDone) {
		MahonyAHRSupdateInitialOrientation(accelXYZ, magXYZ, att);
		att->initialUpdateDone = 1;
	}

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		MahonyAHRSupdateIMU(gyroXYZ, accelXYZ, dt, att);
		return;
	}

	accelNorm = sqrtf(ax * ax + ay * ay + az * az);

	// Compute feedback only if accelerometer abs(vector)is not too small to avoid a division
	// by a small number
	if (accelNorm > 0.01) {
		accelConfidence = calculateAccConfidence(accelNorm, &att->accMagP);
		twoKp *= accelConfidence;
		twoKi *= accelConfidence;

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		q0q0 = att->q0 * att->q0;
		q0q1 = att->q0 * att->q1;
		q0q2 = att->q0 * att->q2;
		q0q3 = att->q0 * att->q3;
		q1q1 = att->q1 * att->q1;
		q1q2 = att->q1 * att->q2;
		q1q3 = att->q1 * att->q3;
		q2q2 = att->q2 * att->q2;
		q2q3 = att->q2 * att->q3;
		q3q3 = att->q3 * att->q3;

		// Reference direction of Earth's magnetic field
		hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
		hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
		bx = sqrtf(hx * hx + hy * hy);
		bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

		// Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
		halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
		halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
		halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			att->integralFBx += twoKi * halfex * dt;	// integral error scaled by Ki
			att->integralFBy += twoKi * halfey * dt;
			att->integralFBz += twoKi * halfez * dt;
			gx += att->integralFBx;	// apply integral feedback
			gy += att->integralFBy;
			gz += att->integralFBz;
		} else {
			att->integralFBx = 0.0f;	// prevent integral windup
			att->integralFBy = 0.0f;
			att->integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}

	// Integrate rate of change of quaternion
	gx *= (0.5f * dt);		// pre-multiply common factors
	gy *= (0.5f * dt);
	gz *= (0.5f * dt);
	qa = att->q0;
	qb = att->q1;
	qc = att->q2;
	att->q0 += (-qb * gx - qc * gy - att->q3 * gz);
	att->q1 += (qa * gx + qc * gz - att->q3 * gy);
	att->q2 += (qa * gy - qb * gz + att->q3 * gx);
	att->q3 += (qa * gz + qb * gy - qc * gx);

	// Normalise quaternion
	recipNorm = invSqrt(att->q0 * att->q0 + att->q1 * att->q1 + att->q2 * att->q2 + att->q3 * att->q3);
	att->q0 *= recipNorm;
	att->q1 *= recipNorm;
	att->q2 *= recipNorm;
	att->q3 *= recipNorm;
}

// IMU algorithm update
void MahonyAHRSupdateIMU(float *gyroXYZ, float *accelXYZ, float dt, ATTITUDE_INFO *att) {
	float accelNorm, recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	float gx = gyroXYZ[0];
	float gy = gyroXYZ[1];
	float gz = gyroXYZ[2];

	float ax = accelXYZ[0];
	float ay = accelXYZ[1];
	float az = accelXYZ[2];

	volatile float twoKp = TWO_KP;
	volatile float twoKi = TWO_KI;
	float accelConfidence;

	accelNorm = sqrtf(ax * ax + ay * ay + az * az);

	// Compute feedback only if accelerometer abs(vector)is not too small to avoid a division
	// by a small number
	if (accelNorm > 0.01) {
		accelConfidence = calculateAccConfidence(accelNorm, &att->accMagP);
		twoKp *= accelConfidence;
		twoKi *= accelConfidence;

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = att->q1 * att->q3 - att->q0 * att->q2;
		halfvy = att->q0 * att->q1 + att->q2 * att->q3;
		halfvz = att->q0 * att->q0 - 0.5f + att->q3 * att->q3;

		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			att->integralFBx += twoKi * halfex * dt;	// integral error scaled by Ki
			att->integralFBy += twoKi * halfey * dt;
			att->integralFBz += twoKi * halfez * dt;
			gx += att->integralFBx;	// apply integral feedback
			gy += att->integralFBy;
			gz += att->integralFBz;
		} else {
			att->integralFBx = 0.0f;	// prevent integral windup
			att->integralFBy = 0.0f;
			att->integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}

	// Integrate rate of change of quaternion
	gx *= (0.5f * dt);		// pre-multiply common factors
	gy *= (0.5f * dt);
	gz *= (0.5f * dt);
	qa = att->q0;
	qb = att->q1;
	qc = att->q2;
	att->q0 += (-qb * gx - qc * gy - att->q3 * gz);
	att->q1 += (qa * gx + qc * gz - att->q3 * gy);
	att->q2 += (qa * gy - qb * gz + att->q3 * gx);
	att->q3 += (qa * gz + qb * gy - qc * gx);

	// Normalize quaternion
	recipNorm = invSqrt(att->q0 * att->q0 + att->q1 * att->q1 + att->q2 * att->q2 + att->q3 * att->q3);
	att->q0 *= recipNorm;
	att->q1 *= recipNorm;
	att->q2 *= recipNorm;
	att->q3 *= recipNorm;
}

float MahonyAHRSGetRoll(ATTITUDE_INFO *att) {
	float gx, gy, gz;
	gx = 2 * (att->q1 *  att->q3 - att->q0 * att->q2);
	gy = 2 * (att->q0 * att->q1 + att->q2 * att->q3);
	gz = att->q0 * att->q0 - att->q1 * att->q1 - att->q2 * att->q2 + att->q3 * att->q3;

	return atanf(gx / sqrtf(gy * gy + gz * gz));
}

float MahonyAHRSGetPitch(ATTITUDE_INFO *att) {
	float gx, gy, gz;
	gx = 2 * (att->q1 *  att->q3 - att->q0 * att->q2);
	gy = 2 * (att->q0 * att->q1 + att->q2 * att->q3);
	gz = att->q0 * att->q0 - att->q1 * att->q1 - att->q2 * att->q2 + att->q3 * att->q3;

	return -atanf(gy / sqrtf(gx * gx + gz * gz));
}

float MahonyAHRSGetYaw(ATTITUDE_INFO *att) {
	return -atan2f(2 * att->q1 * att->q2 - 2 * att->q0 * att->q3,
			2 * att->q0 * att->q0 + 2 *att->q1 * att->q1 - 1);
}

void MahonyAHRSGetRollPitchYaw(float *rpy, ATTITUDE_INFO *att) {
	float gx, gy, gz;
	gx = 2 * (att->q1 *  att->q3 - att->q0 * att->q2);
	gy = 2 * (att->q0 * att->q1 + att->q2 * att->q3);
	gz = att->q0 * att->q0 - att->q1 * att->q1 - att->q2 * att->q2 + att->q3 * att->q3;

	rpy[0] = atanf(gx / sqrtf(gy * gy + gz * gz));
	rpy[1] = -atanf(gy / sqrtf(gx * gx + gz * gz));
	rpy[2] = -atan2f(2 * att->q1 * att->q2 - 2 * att->q0 * att->q3,
			2 * att->q0 * att->q0 + 2 *att->q1 * att->q1 - 1);
}

// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
static float invSqrt(float x) {
	union {
		float as_float;
		long as_int;
	} un;

	float xhalf = 0.5f*x;
	un.as_float = x;
	un.as_int = 0x5f3759df - (un.as_int >> 1);
	un.as_float = un.as_float * (1.5f - xhalf * un.as_float * un.as_float);
	return un.as_float;
}
