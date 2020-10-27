#include <math.h>
#include <stdbool.h>

#include "sensfusion9.h"

#pragma once

#define SPEED_OF_LIGHT (299792458.0)
#define GRAVITY_MAGNITUDE (9.81f)

#ifndef M_PI
  #define M_PI   3.14159265358979323846
#endif

#ifndef M_PI_F
  #define M_PI_F   (3.14159265358979323846f)
#endif

#ifndef M_1_PI_F
  #define M_1_PI_F (0.31830988618379067154f)
#endif

#ifndef M_PI_2_F
  #define M_PI_2_F (1.57079632679f)
#endif

float twoKp = (10.0f * 0.4f);  
float twoKi = (10.0f * 0.001f);
float integralFBx = 0.0f;
float integralFBy = 0.0f;
float integralFBz = 0.0f; 

float q0 = 1.0f;
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;  

static float gravX, gravY, gravZ; 
static float baseZacc = 1.0;
static bool isInit;
static bool isCalibrated = false;

static void sensfusion9UpdateImpl(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt);
static float sensfusion9GetAccZ(const float ax, const float ay, const float az);
static void estimatedGravityDirection(float* gx, float* gy, float* gz);

static float invSqrt(float x);
static int reset_next_update=0;

void sensfusion9Init()
{
  if(isInit)
    return;
  reset_next_update = 1;
  isInit = true;
}

void sensfusion9Update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt)
{
  sensfusion9UpdateImpl(gx, gy, gz, ax, ay, az, mx, my, mz, dt);
  estimatedGravityDirection(&gravX, &gravY, &gravZ);

  if (!isCalibrated) {
    baseZacc = sensfusion9GetAccZ(ax, ay, az);
    isCalibrated = true;
  }
}

// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/open-source-ahrs-with-x-imu
//
// Date     Author      Notes
// 29/09/2011 SOH Madgwick    Initial release
// 02/10/2011 SOH Madgwick  Optimised for reduced CPU load
static void sensfusion9UpdateImpl(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt)
{
  float recipNorm;
	float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Compute feedback only if accelerometer measurement valid
	// (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

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
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;

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

		// Error is sum of cross product between estimated direction
		// and measured direction of field vectors
		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			// integral error scaled by Ki
			integralFBx += twoKi * halfex * dt;
			integralFBy += twoKi * halfey * dt;
			integralFBz += twoKi * halfez * dt;
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		} else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		if (reset_next_update) {
			gx += 2.0f * halfex;
			gy += 2.0f * halfey;
			gz += 2.0f * halfez;
			reset_next_update = 0;
		} else {
			gx += twoKp * halfex;
			gy += twoKp * halfey;
			gz += twoKp * halfez;
		}
	}

	// Integrate rate of change of quaternion
	gx *= (0.5f * dt);		// pre-multiply common factors
	gy *= (0.5f * dt);
	gz *= (0.5f * dt);
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

void sensfusion9GetQuaternion(float* q)
{
  q[0] = q0;
  q[1] = q1;
  q[2] = q2;
  q[3] = q3;
}

void sensfusion9GetEulerRPY(float* angles)
{
  double gx = gravX;
  double gy = gravY;
  double gz = gravZ;

  if (gx>1) gx=1;
  if (gx<-1) gx=-1;

  angles[0] =  	(double) atan2f(2*(q0*q3 + q1*q2), q0*q0 + q1*q1 - q2*q2 - q3*q3) * 180 / M_PI_F;
  angles[1] =  (double) asinf(gx) * 180 / M_PI_F; //Pitch seems to be inverted
  angles[2] =   (double) atan2f(gy, gz) * 180 / M_PI_F;
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
float invSqrt(float x)
{
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}

static float sensfusion9GetAccZ(const float ax, const float ay, const float az)
{
  // return vertical acceleration
  // (A dot G) / |G|,  (|G| = 1) -> (A dot G)
  return (ax * gravX + ay * gravY + az * gravZ);
}

static void estimatedGravityDirection(float* gx, float* gy, float* gz)
{
  *gx = 2 * (q1 * q3 - q0 * q2);
  *gy = 2 * (q0 * q1 + q2 * q3);
  *gz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
}
