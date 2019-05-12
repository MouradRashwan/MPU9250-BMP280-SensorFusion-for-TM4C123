/*
 * SensorFusion_defs.h
 *
 *  Created on: Mar 19, 2019
 *      Author: Administrator
 */

#ifndef SENSORFUSION_DEFS_H_
#define SENSORFUSION_DEFS_H_

#ifndef PI
#define PI      3.14159265358979323846  /* pi */
#endif

/* for Mahony algorithm */
typedef struct PIgain
{
    float dKp;
    float dKi;
} PIgain_t;

/* for Mahony algorithm */
typedef struct IntegralError
{
    float dX;
    float dY;
    float dZ;
} IntegralError_t;

typedef struct Quaternion
{
    float dQ1;
    float dQ2;
    float dQ3;
    float dQ4;
} Quaternion_t;

typedef struct YawPitchRoll
{
    float dYaw;
    float dPitch;
    float dRoll;
} YawPitchRoll_t;

typedef struct SensorFusion9Axis
{
    float dAx;
    float dAy;
    float dAz;
    float dGx;
    float dGy;
    float dGz;
    float dMx;
    float dMy;
    float dMz;
} SensorFusion9Axis_t;

#endif /* SENSORFUSION_DEFS_H_ */
