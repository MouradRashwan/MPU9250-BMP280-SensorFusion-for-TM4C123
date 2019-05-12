/*
 * SensorFusion.h
 *
 *  Created on: Mar 19, 2019
 *      Author: Administrator
 */

#ifndef SENSORFUSION_H_
#define SENSORFUSION_H_

// Mahony algorithm is similat to Madgwick scheme but uses proportional and integral filtering
// on the error between estimated reference vectors and
// measured ones.
// Kp = 10.0
// Ki = 0.0

// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!

// Sensors x (y)-axis of the accelerometer/gyro is aligned with the y (x)-axis of the magnetometer;
// the magnetometer z-axis (+ down) is misaligned with z-axis (+ up) of accelerometer and gyro!
// We have to make some allowance for this orientation mismatch in feeding the output to the quaternion filter.
// For the MPU9250+MS5637 Mini breakout the +x accel/gyro is North, then -y accel/gyro is East. So if we want te quaternions properly aligned
// we need to feed into the Madgwick function Ax, -Ay, -Az, Gx, -Gy, -Gz, My, -Mx, and Mz. But because gravity is by convention
// positive down, we need to invert the accel data, so we pass -Ax, Ay, Az, Gx, -Gy, -Gz, My, -Mx, and Mz into the Madgwick
// function to get North along the accel +x-axis, East along the accel -y-axis, and Down along the accel -z-axis.
// This orientation choice can be modified to allow any convenient (non-NED) orientation convention.
// Pass gyro rate as rad/s

void SensorFusion_initQuaternion(Quaternion_t *ptQuaternion);

/* Consume more power - Take more time - Give more accurate results */
void SensorFusion_MadgwickUpdate(Quaternion_t *ptQuaternion,
                                 SensorFusion9Axis_t *ptSensorFusion9Axis,
                                 float dGyroMeasError, float dPeriodInSec);

/* Consume less power - Take less time - Give less accurate results */
void SensorFusion_MahonyUpdate(Quaternion_t *ptQuaternion,
                               SensorFusion9Axis_t *ptSensorFusion9Axis,
                               PIgain_t tPIgain, float dPeriodInSec);

/* That's the final step to get your angles */
void SensorFusion_getYawPitchRoll(YawPitchRoll_t *ptYawPitchRoll,
                                  Quaternion_t *ptQuaternion);

#endif /* SENSORFUSION_H_ */
