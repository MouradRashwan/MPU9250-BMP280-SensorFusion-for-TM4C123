/*
 * SensorFusion.c
 *
 *  Created on: Mar 19, 2019
 *      Author: Administrator
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include <math.h>

#include "SensorFusion_driver.h"

void SensorFusion_initQuaternion(Quaternion_t *ptQuaternion)
{
    ptQuaternion->dQ1 = 1.0f;
    ptQuaternion->dQ2 = 0.0f;
    ptQuaternion->dQ3 = 0.0f;
    ptQuaternion->dQ4 = 0.0f;
}

/* Consume more power - Take more time - Give more accurate results */
void SensorFusion_MadgwickUpdate(Quaternion_t *ptQuaternion,
                                 SensorFusion9Axis_t *ptSensorFusion9Axis,
                                 float dGyroMeasError, float dPeriodInSec)
{
    // short name local variable for readability
    float q1, q2, q3, q4;
    float norm, beta;
    float hx, hy, _2bx, _2bz;
    float s1, s2, s3, s4;
    float qDot1, qDot2, qDot3, qDot4;

    // Auxiliary variables to avoid repeated arithmetic
    float _2q1mx;
    float _2q1my;
    float _2q1mz;
    float _2q2mx;
    float _4bx;
    float _4bz;
    float _2q1;
    float _2q2;
    float _2q3;
    float _2q4;
    float _2q1q3;
    float _2q3q4;
    float q1q1;
    float q1q2;
    float q1q3;
    float q1q4;
    float q2q2;
    float q2q3;
    float q2q4;
    float q3q3;
    float q3q4;
    float q4q4;

    q1 = ptQuaternion->dQ1;
    q2 = ptQuaternion->dQ2;
    q3 = ptQuaternion->dQ3;
    q4 = ptQuaternion->dQ4;

    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _2q4 = 2.0f * q4;
    _2q1q3 = 2.0f * q1 * q3;
    _2q3q4 = 2.0f * q3 * q4;

    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q1q4 = q1 * q4;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q2q4 = q2 * q4;
    q3q3 = q3 * q3;
    q3q4 = q3 * q4;
    q4q4 = q4 * q4;

    // compute beta
    beta = sqrtf(3.0f / 4.0f) * dGyroMeasError;

    // Normalise accelerometer measurement
    norm = sqrtf(
            ptSensorFusion9Axis->dAx * ptSensorFusion9Axis->dAx
                    + ptSensorFusion9Axis->dAy * ptSensorFusion9Axis->dAy
                    + ptSensorFusion9Axis->dAz * ptSensorFusion9Axis->dAz);

    if (norm == 0.0f)
    {
        return; // handle NaN
    }

    norm = 1.0f / norm;
    ptSensorFusion9Axis->dAx *= norm;
    ptSensorFusion9Axis->dAy *= norm;
    ptSensorFusion9Axis->dAz *= norm;

    // Normalise magnetometer measurement
    norm = sqrtf(
            ptSensorFusion9Axis->dMx * ptSensorFusion9Axis->dMx
                    + ptSensorFusion9Axis->dMy * ptSensorFusion9Axis->dMy
                    + ptSensorFusion9Axis->dMz * ptSensorFusion9Axis->dMz);

    if (norm == 0.0f)
    {
        return; // handle NaN
    }

    norm = 1.0f / norm;
    ptSensorFusion9Axis->dMx *= norm;
    ptSensorFusion9Axis->dMy *= norm;
    ptSensorFusion9Axis->dMz *= norm;

    // Reference direction of Earth's magnetic field
    _2q1mx = 2.0f * q1 * ptSensorFusion9Axis->dMx;
    _2q1my = 2.0f * q1 * ptSensorFusion9Axis->dMy;
    _2q1mz = 2.0f * q1 * ptSensorFusion9Axis->dMz;
    _2q2mx = 2.0f * q2 * ptSensorFusion9Axis->dMx;
    hx = ptSensorFusion9Axis->dMx * q1q1 - _2q1my * q4 + _2q1mz * q3
            + ptSensorFusion9Axis->dMx * q2q2
            + _2q2 * ptSensorFusion9Axis->dMy * q3
            + _2q2 * ptSensorFusion9Axis->dMz * q4
            - ptSensorFusion9Axis->dMx * q3q3 - ptSensorFusion9Axis->dMx * q4q4;
    hy = _2q1mx * q4 + ptSensorFusion9Axis->dMy * q1q1 - _2q1mz * q2
            + _2q2mx * q3 - ptSensorFusion9Axis->dMy * q2q2
            + ptSensorFusion9Axis->dMy * q3q3
            + _2q3 * ptSensorFusion9Axis->dMz * q4
            - ptSensorFusion9Axis->dMy * q4q4;
    _2bx = sqrtf(hx * hx + hy * hy);
    _2bz = -_2q1mx * q3 + _2q1my * q2 + ptSensorFusion9Axis->dMz * q1q1
            + _2q2mx * q4 - ptSensorFusion9Axis->dMz * q2q2
            + _2q3 * ptSensorFusion9Axis->dMy * q4
            - ptSensorFusion9Axis->dMz * q3q3 + ptSensorFusion9Axis->dMz * q4q4;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ptSensorFusion9Axis->dAx)
            + _2q2 * (2.0f * q1q2 + _2q3q4 - ptSensorFusion9Axis->dAy)
            - _2bz * q3
                    * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3)
                            - ptSensorFusion9Axis->dMx)
            + (-_2bx * q4 + _2bz * q2)
                    * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4)
                            - ptSensorFusion9Axis->dMy)
            + _2bx * q3
                    * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3)
                            - ptSensorFusion9Axis->dMz);
    s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ptSensorFusion9Axis->dAx)
            + _2q1 * (2.0f * q1q2 + _2q3q4 - ptSensorFusion9Axis->dAy)
            - 4.0f * q2
                    * (1.0f - 2.0f * q2q2 - 2.0f * q3q3
                            - ptSensorFusion9Axis->dAz)
            + _2bz * q4
                    * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3)
                            - ptSensorFusion9Axis->dMx)
            + (_2bx * q3 + _2bz * q1)
                    * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4)
                            - ptSensorFusion9Axis->dMy)
            + (_2bx * q4 - _4bz * q2)
                    * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3)
                            - ptSensorFusion9Axis->dMz);
    s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ptSensorFusion9Axis->dAx)
            + _2q4 * (2.0f * q1q2 + _2q3q4 - ptSensorFusion9Axis->dAy)
            - 4.0f * q3
                    * (1.0f - 2.0f * q2q2 - 2.0f * q3q3
                            - ptSensorFusion9Axis->dAz)
            + (-_4bx * q3 - _2bz * q1)
                    * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3)
                            - ptSensorFusion9Axis->dMx)
            + (_2bx * q2 + _2bz * q4)
                    * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4)
                            - ptSensorFusion9Axis->dMy)
            + (_2bx * q1 - _4bz * q3)
                    * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3)
                            - ptSensorFusion9Axis->dMz);
    s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ptSensorFusion9Axis->dAx)
            + _2q3 * (2.0f * q1q2 + _2q3q4 - ptSensorFusion9Axis->dAy)
            + (-_4bx * q4 + _2bz * q2)
                    * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3)
                            - ptSensorFusion9Axis->dMx)
            + (-_2bx * q1 + _2bz * q3)
                    * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4)
                            - ptSensorFusion9Axis->dMy)
            + _2bx * q2
                    * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3)
                            - ptSensorFusion9Axis->dMz);

    // normalise step magnitude
    norm = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);
    norm = 1.0f / norm;
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;
    s4 *= norm;

    // Compute rate of change of quaternion
    qDot1 = 0.5f
            * (-q2 * ptSensorFusion9Axis->dGx - q3 * ptSensorFusion9Axis->dGy
                    - q4 * ptSensorFusion9Axis->dGz) - beta * s1;
    qDot2 = 0.5f
            * (q1 * ptSensorFusion9Axis->dGx + q3 * ptSensorFusion9Axis->dGz
                    - q4 * ptSensorFusion9Axis->dGy) - beta * s2;
    qDot3 = 0.5f
            * (q1 * ptSensorFusion9Axis->dGy - q2 * ptSensorFusion9Axis->dGz
                    + q4 * ptSensorFusion9Axis->dGx) - beta * s3;
    qDot4 = 0.5f
            * (q1 * ptSensorFusion9Axis->dGz + q2 * ptSensorFusion9Axis->dGy
                    - q3 * ptSensorFusion9Axis->dGx) - beta * s4;

    // Integrate to yield quaternion
    q1 += qDot1 * dPeriodInSec;
    q2 += qDot2 * dPeriodInSec;
    q3 += qDot3 * dPeriodInSec;
    q4 += qDot4 * dPeriodInSec;

    // normalise quaternion
    norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
    norm = 1.0f / norm;

    ptQuaternion->dQ1 = q1 * norm;
    ptQuaternion->dQ2 = q2 * norm;
    ptQuaternion->dQ3 = q3 * norm;
    ptQuaternion->dQ4 = q4 * norm;
}

/* Consume less power - Take less time - Give less accurate results */
void SensorFusion_MahonyUpdate(Quaternion_t *ptQuaternion,
                               SensorFusion9Axis_t *ptSensorFusion9Axis,
                               PIgain_t tPIgain, float dPeriodInSec)
{
    static IntegralError_t s_tIntegralError = { 0.0f, 0.0f, 0.0f };

    float q1, q2, q3, q4; // short name local variable for readability
    float norm;
    float hx, hy, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float ex, ey, ez;
    float pa, pb, pc;

    // Auxiliary variables to avoid repeated arithmetic
    float q1q1;
    float q1q2;
    float q1q3;
    float q1q4;
    float q2q2;
    float q2q3;
    float q2q4;
    float q3q3;
    float q3q4;
    float q4q4;

    q1 = ptQuaternion->dQ1;
    q2 = ptQuaternion->dQ2;
    q3 = ptQuaternion->dQ3;
    q4 = ptQuaternion->dQ4;

    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q1q4 = q1 * q4;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q2q4 = q2 * q4;
    q3q3 = q3 * q3;
    q3q4 = q3 * q4;
    q4q4 = q4 * q4;

    // Normalise accelerometer measurement
    norm = sqrtf(
            ptSensorFusion9Axis->dAx * ptSensorFusion9Axis->dAx
                    + ptSensorFusion9Axis->dAy * ptSensorFusion9Axis->dAy
                    + ptSensorFusion9Axis->dAz * ptSensorFusion9Axis->dAz);

    if (norm == 0.0f)
    {
        return; // handle NaN
    }

    norm = 1.0f / norm;        // use reciprocal for division
    ptSensorFusion9Axis->dAx *= norm;
    ptSensorFusion9Axis->dAy *= norm;
    ptSensorFusion9Axis->dAz *= norm;

    // Normalise magnetometer measurement
    norm = sqrtf(
            ptSensorFusion9Axis->dMx * ptSensorFusion9Axis->dMx
                    + ptSensorFusion9Axis->dMy * ptSensorFusion9Axis->dMy
                    + ptSensorFusion9Axis->dMz * ptSensorFusion9Axis->dMz);

    if (norm == 0.0f)
    {
        return; // handle NaN
    }

    norm = 1.0f / norm;        // use reciprocal for division
    ptSensorFusion9Axis->dMx *= norm;
    ptSensorFusion9Axis->dMy *= norm;
    ptSensorFusion9Axis->dMz *= norm;

    // Reference direction of Earth's magnetic field
    hx = 2.0f * ptSensorFusion9Axis->dMx * (0.5f - q3q3 - q4q4)
            + 2.0f * ptSensorFusion9Axis->dMy * (q2q3 - q1q4)
            + 2.0f * ptSensorFusion9Axis->dMz * (q2q4 + q1q3);
    hy = 2.0f * ptSensorFusion9Axis->dMx * (q2q3 + q1q4)
            + 2.0f * ptSensorFusion9Axis->dMy * (0.5f - q2q2 - q4q4)
            + 2.0f * ptSensorFusion9Axis->dMz * (q3q4 - q1q2);
    bx = sqrtf((hx * hx) + (hy * hy));
    bz = 2.0f * ptSensorFusion9Axis->dMx * (q2q4 - q1q3)
            + 2.0f * ptSensorFusion9Axis->dMy * (q3q4 + q1q2)
            + 2.0f * ptSensorFusion9Axis->dMz * (0.5f - q2q2 - q3q3);

    // Estimated direction of gravity and magnetic field
    vx = 2.0f * (q2q4 - q1q3);
    vy = 2.0f * (q1q2 + q3q4);
    vz = q1q1 - q2q2 - q3q3 + q4q4;
    wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
    wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
    wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

    // Error is cross product between estimated direction and measured direction of gravity
    ex = (ptSensorFusion9Axis->dAy * vz - ptSensorFusion9Axis->dAz * vy)
            + (ptSensorFusion9Axis->dMy * wz - ptSensorFusion9Axis->dMz * wy);
    ey = (ptSensorFusion9Axis->dAz * vx - ptSensorFusion9Axis->dAx * vz)
            + (ptSensorFusion9Axis->dMz * wx - ptSensorFusion9Axis->dMx * wz);
    ez = (ptSensorFusion9Axis->dAx * vy - ptSensorFusion9Axis->dAy * vx)
            + (ptSensorFusion9Axis->dMx * wy - ptSensorFusion9Axis->dMy * wx);

    if (tPIgain.dKi > 0.0f)
    {
        s_tIntegralError.dX += ex;      // accumulate integral error
        s_tIntegralError.dY += ey;
        s_tIntegralError.dZ += ez;
    }
    else
    {
        s_tIntegralError.dX = 0.0f;     // prevent integral wind up
        s_tIntegralError.dY = 0.0f;
        s_tIntegralError.dZ = 0.0f;
    }

    // Apply feedback terms
    ptSensorFusion9Axis->dGx = ptSensorFusion9Axis->dGx + tPIgain.dKp * ex
            + tPIgain.dKi * s_tIntegralError.dX;
    ptSensorFusion9Axis->dGy = ptSensorFusion9Axis->dGy + tPIgain.dKp * ey
            + tPIgain.dKi * s_tIntegralError.dY;
    ptSensorFusion9Axis->dGz = ptSensorFusion9Axis->dGz + tPIgain.dKp * ez
            + tPIgain.dKi * s_tIntegralError.dZ;

    // Integrate rate of change of quaternion
    pa = q2;
    pb = q3;
    pc = q4;
    q1 = q1
            + (-q2 * ptSensorFusion9Axis->dGx - q3 * ptSensorFusion9Axis->dGy
                    - q4 * ptSensorFusion9Axis->dGz) * (0.5f * dPeriodInSec);
    q2 = pa
            + (q1 * ptSensorFusion9Axis->dGx + pb * ptSensorFusion9Axis->dGz
                    - pc * ptSensorFusion9Axis->dGy) * (0.5f * dPeriodInSec);
    q3 = pb
            + (q1 * ptSensorFusion9Axis->dGy - pa * ptSensorFusion9Axis->dGz
                    + pc * ptSensorFusion9Axis->dGx) * (0.5f * dPeriodInSec);
    q4 = pc
            + (q1 * ptSensorFusion9Axis->dGz + pa * ptSensorFusion9Axis->dGy
                    - pb * ptSensorFusion9Axis->dGx) * (0.5f * dPeriodInSec);

    // Normalise quaternion
    norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
    norm = 1.0f / norm;

    ptQuaternion->dQ1 = q1 * norm;
    ptQuaternion->dQ2 = q2 * norm;
    ptQuaternion->dQ3 = q3 * norm;
    ptQuaternion->dQ4 = q4 * norm;
}

void SensorFusion_getYawPitchRoll(YawPitchRoll_t *ptYawPitchRoll,
                                  Quaternion_t *ptQuaternion)
{
    ptYawPitchRoll->dYaw = atan2f(
            2.0f
                    * (ptQuaternion->dQ2 * ptQuaternion->dQ3
                            + ptQuaternion->dQ1 * ptQuaternion->dQ4),
            ptQuaternion->dQ1 * ptQuaternion->dQ1
                    + ptQuaternion->dQ2 * ptQuaternion->dQ2
                    - ptQuaternion->dQ3 * ptQuaternion->dQ3
                    - ptQuaternion->dQ4 * ptQuaternion->dQ4);

    ptYawPitchRoll->dPitch = -asinf(
            2.0f
                    * (ptQuaternion->dQ2 * ptQuaternion->dQ4
                            - ptQuaternion->dQ1 * ptQuaternion->dQ3));

    ptYawPitchRoll->dRoll = atan2f(
            2.0f
                    * (ptQuaternion->dQ1 * ptQuaternion->dQ2
                            + ptQuaternion->dQ3 * ptQuaternion->dQ4),
            ptQuaternion->dQ1 * ptQuaternion->dQ1
                    - ptQuaternion->dQ2 * ptQuaternion->dQ2
                    - ptQuaternion->dQ3 * ptQuaternion->dQ3
                    + ptQuaternion->dQ4 * ptQuaternion->dQ4);

    ptYawPitchRoll->dYaw *= 180.0f / PI;
//    ptYawPitchRoll->dYaw += 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
//    if (ptYawPitchRoll->dYaw < 0)
//    {
//        ptYawPitchRoll->dYaw += 360.0f; // Ensure yaw stays between 0 and 360
//    }

    ptYawPitchRoll->dPitch *= 180.0f / PI;

    ptYawPitchRoll->dRoll *= 180.0f / PI;
}

