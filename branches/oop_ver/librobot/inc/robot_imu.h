/*
 * robot_imu.h
 *
 *  Created on: Feb 15, 2015
 *      Author: VyLong
 */

#ifndef ROBOT_IMU_H_
#define ROBOT_IMU_H_

#include <stdint.h>
#include <stdbool.h>
#include "libcustom/inc/custom_i2c.h"
#include "eMPL/inc/inv_mpu.h"

#include "libmath/inc/quaternion.h"
#include "libmath/inc/vector3.h"

#define DEFAULT_MPU_HZ  (100)
#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)
#define q30  			1073741824.0f

void initIMU(InvMPU* pMpu6050);

Vector3<float> IMU_getWorldLinearAccel(void);
Vector3<float> IMU_getLinearAccel(void);
Vector3<float> IMU_getYawPitchRoll(void);
Vector3<float> IMU_getGravity(void);
Vector3<float> IMU_getEulerAngles(void);
Quaternion IMU_getQuaternion(void);
void IMU_updateNewRaw(void);

void IMU_convertQuaternionToEulerAngles(Vector3<float> *pvect3Euler, Quaternion q);
void IMU_convertQuaternionToGravity(Vector3<float> *pvect3Gravity, Quaternion q);
void IMU_convertQuaternionAndGravityToYawPitchRoll(Vector3<float> *pvect3YPR, Quaternion q, Vector3<float> vect3Gravity);
void IMU_convertAccelerationAndGravityToLinearAccel(Vector3<float> *pvect3LinearAccel, Vector3<short> vect3AccelRaw, Vector3<float> vect3Gravity);
void IMU_convertLinearAccelAndQuaternionToWorldLinearAccel(Vector3<float> *pvect3WorldLinearAccel, Vector3<float> vect3LinearAccel, Quaternion q);

void IMU_run_self_test(InvMPU* pMpu6050);
void IMU_setup_mpu_dmp(InvMPU* pMpu6050);

unsigned short IMU_inv_row_2_scale(const signed char *row);
unsigned short IMU_inv_orientation_matrix_to_scalar(const signed char *mtx);

#endif /* ROBOT_IMU_H_ */
