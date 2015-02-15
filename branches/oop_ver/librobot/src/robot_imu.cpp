/*
 * robot_imu.c
 *
 *  Created on: Feb 15, 2015
 *      Author: VyLong
 */

#include "librobot/inc/robot_imu.h"
#include "libcustom/inc/custom_uart_debug.h"

InvMPU* g_mpu6050;

unsigned long sensor_timestamp;

long quat[4];
short gyro[3];
short accel[3];

unsigned char more;	// Temperature

static signed char gyro_orientation[9] = { -1, 0, 0,
											0, -1, 0,
											0, 0, 1 };

void initIMU(InvMPU* pMpu6050)
{
	g_mpu6050 = pMpu6050;
	IMU_setup_mpu_dmp(pMpu6050);
	pMpu6050->mpu_reset_fifo();
}


Vector3<float> IMU_getWorldLinearAccel(void)
{
	Vector3<float> vect3WorldLinearAccel;

	Vector3<float> vect3LinearAccel;

	Vector3<float> vect3Gravity;

	Quaternion q;
	//NOTE: this function below spin about 60-100ms
	IMU_updateNewRaw();

	q.w = quat[0] / q30;
	q.x = quat[1] / q30;
	q.y = quat[2] / q30;
	q.z = quat[3] / q30;

	Vector3<short> vect3AccelRaw(accel[0], accel[1], accel[2]);

	IMU_convertQuaternionToGravity(&vect3Gravity, q);

	IMU_convertAccelerationAndGravityToLinearAccel(&vect3LinearAccel, vect3AccelRaw, vect3Gravity);

	IMU_convertLinearAccelAndQuaternionToWorldLinearAccel(&vect3WorldLinearAccel, vect3LinearAccel, q);

	return vect3WorldLinearAccel;
}

Vector3<float> IMU_getLinearAccel(void)
{
	Vector3<float> vect3LinearAccel;

	Vector3<float> vect3Gravity;

	Quaternion q;
	//NOTE: this function below spin about 60-100ms
	IMU_updateNewRaw();

	q.w = quat[0] / q30;
	q.x = quat[1] / q30;
	q.y = quat[2] / q30;
	q.z = quat[3] / q30;

	Vector3<short> vect3AccelRaw(accel[0], accel[1], accel[2]);

	IMU_convertQuaternionToGravity(&vect3Gravity, q);

	IMU_convertAccelerationAndGravityToLinearAccel(&vect3LinearAccel, vect3AccelRaw, vect3Gravity);

	return vect3LinearAccel;
}

Vector3<float> IMU_getYawPitchRoll(void)
{
	Vector3<float> vect3YawPitchRoll;

	Vector3<float> vect3Gravity;

	Quaternion q = IMU_getQuaternion();

	IMU_convertQuaternionToGravity(&vect3Gravity, q);

	IMU_convertQuaternionAndGravityToYawPitchRoll(&vect3YawPitchRoll, q, vect3Gravity);

	return vect3YawPitchRoll;
}

Vector3<float> IMU_getGravity(void)
{
	Vector3<float> vect3Gravity;

	Quaternion q = IMU_getQuaternion();

	IMU_convertQuaternionToGravity(&vect3Gravity, q);

	return vect3Gravity;
}

Vector3<float> IMU_getEulerAngles(void)
{
	Vector3<float> vect3EulerAngles;

	Quaternion q = IMU_getQuaternion();

	IMU_convertQuaternionToEulerAngles(&vect3EulerAngles, q);

	return vect3EulerAngles;
}

Quaternion IMU_getQuaternion(void)
{
	Quaternion q;
	//NOTE: this function below spin about 60-100ms
	IMU_updateNewRaw();

	q.w = quat[0] / q30;
	q.x = quat[1] / q30;
	q.y = quat[2] / q30;
	q.z = quat[3] / q30;

	return q;
}


void IMU_updateNewRaw(void)
{
	short sensors;

	while(true)
	{
		/* This function gets new data from the FIFO when the DMP is in
		 * use. The FIFO can contain any combination of gyro, accel,
		 * quaternion, and gesture data. The sensors parameter tells the
		 * caller which data fields were actually populated with new data.
		 * For example, if sensors == (INV_XYZ_GYRO | INV_WXYZ_QUAT), then
		 * the FIFO isn't being filled with accel data.
		 * The driver parses the gesture data to determine if a gesture
		 * event has occurred; on an event, the application will be notified
		 * via a callback (assuming that a callback function was properly
		 * registered). The more parameter is non-zero if there are
		 * leftover packets in the FIFO.
		 */
		g_mpu6050->dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);

		/* Gyro and accel data are written to the FIFO by the DMP in chip
		 * frame and hardware units. This behavior is convenient because it
		 * keeps the gyro and accel outputs of dmp_read_fifo and
		 * mpu_read_fifo consistent.
		 */

		if ((sensors & INV_WXYZ_QUAT) && (sensors & INV_XYZ_GYRO) && (sensors & INV_XYZ_ACCEL))
		{
			break;
		}

		delay_ms_i2c(1);
	}
}


void IMU_convertQuaternionToEulerAngles(Vector3<float> *pvect3Euler, Quaternion q)
{
	// Transfrom Fomulator:
	// psi = atan2(2xy - 2wz; 2w^2 + 2x^2 - 1)
	// theta = -asin(2xz + 2wy)
	// phi = atan2(2yz - 2wx; 2w^2 + 2z^2 - 1)

	pvect3Euler->x = atan2f(2*q.x*q.y - 2*q.w*q.z, 2*q.w*q.w + 2*q.x*q.x - 1);	// psi
	pvect3Euler->y = -asinf(2*q.x*q.z + 2*q.w*q.y);	// theta
	pvect3Euler->z = atan2f(2*q.y*q.z - 2*q.w*q.x, 2*q.w*q.w + 2*q.z*q.z - 1);	// phi
}

void IMU_convertQuaternionToGravity(Vector3<float> *pvect3Gravity, Quaternion q)
{
	// Transfrom Fomulator:
	// gx = 2(xz - wy)
	// gy = 2(wx + yz)
	// gz = w^2 - x^2 - y^2 + z^2

	pvect3Gravity->x = 2 * (q.x*q.z - q.w*q.y);
	pvect3Gravity->y = 2 * (q.w*q.x + q.y*q.z);
	pvect3Gravity->z = q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z;
}

void IMU_convertQuaternionAndGravityToYawPitchRoll(Vector3<float> *pvect3YPR, Quaternion q, Vector3<float> vect3Gravity)
{
	// Transfrom Fomulator:
	// yaw = atan2(2qx*qy - 2qw*qz, 2qw^2 + 2qx^2 - 1)
	// pitch = atan(gx / sqrt(gy^2 + gz^2))
	// roll = atan(gy / sqrt(gx^2 + gz^2))

	// yaw: (about Z axis)
	pvect3YPR->x = atan2f(2*q.x*q.y - 2*q.w*q.z, 2*q.w*q.w + 2*q.x*q.x - 1);

    // pitch: (nose up/down, about Y axis)
	pvect3YPR->y = atanf(vect3Gravity.x / sqrtf(vect3Gravity.y*vect3Gravity.y + vect3Gravity.z*vect3Gravity.z));

    // roll: (tilt left/right, about X axis)
	pvect3YPR->z = atanf(vect3Gravity.y / sqrtf(vect3Gravity.x*vect3Gravity.x + vect3Gravity.z*vect3Gravity.z));
}

void IMU_convertAccelerationAndGravityToLinearAccel(Vector3<float> *pvect3LinearAccel, Vector3<short> vect3AccelRaw, Vector3<float> vect3Gravity)
{
	//TODO: confirm gyro and accel sensitivity

    // get rid of the gravity component (+1g = +8192 in standard DMP FIFO packet, sensitivity is 2g)
	pvect3LinearAccel->x = vect3AccelRaw.x - vect3Gravity.x*8192;
	pvect3LinearAccel->y = vect3AccelRaw.y - vect3Gravity.y*8192;
	pvect3LinearAccel->z = vect3AccelRaw.z - vect3Gravity.z*8192;
}

void IMU_convertLinearAccelAndQuaternionToWorldLinearAccel(Vector3<float> *pvect3WorldLinearAccel, Vector3<float> vect3LinearAccel, Quaternion q)
{
    // rotate measured 3D acceleration vector into original state
    // frame of reference based on orientation quaternion
    *pvect3WorldLinearAccel = vect3LinearAccel.getRotated(q);
}


void IMU_run_self_test(InvMPU* pMpu6050)
{
	int result;
	long gyro[3], accel[3];

	result = pMpu6050->mpu_run_self_test(gyro, accel);

	if (result == 0x7)
	{
		//
		// Test passed. We can trust the gyro data here, so let's push it down
		// to the DMP.
		//
		float sens;

		unsigned short accel_sens;

		pMpu6050->mpu_get_gyro_sens(&sens);
		gyro[0] = (long) (gyro[0] * sens);
		gyro[1] = (long) (gyro[1] * sens);
		gyro[2] = (long) (gyro[2] * sens);
		pMpu6050->dmp_set_gyro_bias(gyro);
		pMpu6050->mpu_get_accel_sens(&accel_sens);
		accel[0] *= accel_sens;
		accel[1] *= accel_sens;
		accel[2] *= accel_sens;
		pMpu6050->dmp_set_accel_bias(accel);
	}
	else
	{
	}
}

void IMU_setup_mpu_dmp(InvMPU* pMpu6050)
{
	DEBUG_PRINT("Initialize MPU6050\n");

	if (pMpu6050->mpu_init())
		DEBUG_PRINT("Error in init!!");

	DEBUG_PRINT("mpu initialization complete......\n");

	//
	// Get/set hardware configuration. Start gyro.
	// Wake up all sensors.
	//
	if (!pMpu6050->mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL))
		DEBUG_PRINT("mpu_set_sensor complete ......\n");
	else
		DEBUG_PRINT("mpu_set_sensor come across error ......\n");

	//
	// Push both gyro and accel data into the FIFO.
	//
	if (!pMpu6050->mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL))
		DEBUG_PRINT("mpu_configure_fifo complete ......\n");
	else
		DEBUG_PRINT("mpu_configure_fifo come across error ......\n");

	//
	// mpu_set_sample_rate
	//
	if (!pMpu6050->mpu_set_sample_rate(DEFAULT_MPU_HZ))
		DEBUG_PRINT("mpu_set_sample_rate complete ......\n");
	else
		DEBUG_PRINT("mpu_set_sample_rate error ......\n");

	DEBUG_PRINT("\nInitialize dmp\n\n");
	/* To initialize the DMP:
	 * 1. Call dmp_load_motion_driver_firmware(). This pushes the DMP image in
	 *    inv_mpu_dmp_motion_driver.h into the MPU memory.
	 * 2. Push the gyro and accel orientation matrix to the DMP.
	 * 3. Register gesture callbacks. Don't worry, these callbacks won't be
	 *    executed unless the corresponding feature is enabled.
	 * 4. Call dmp_enable_feature(mask) to enable different features.
	 * 5. Call dmp_set_fifo_rate(freq) to select a DMP output rate.
	 * 6. Call any feature-specific control functions.
	 *
	 * To enable the DMP, just call mpu_set_dmp_state(1). This function can
	 * be called repeatedly to enable and disable the DMP at runtime.
	 *
	 * The following is a short summary of the features supported in the DMP
	 * image provided in inv_mpu_dmp_motion_driver.c:
	 * DMP_FEATURE_LP_QUAT: Generate a gyro-only quaternion on the DMP at
	 * 200Hz. Integrating the gyro data at higher rates reduces numerical
	 * errors (compared to integration on the MCU at a lower sampling rate).
	 * DMP_FEATURE_6X_LP_QUAT: Generate a gyro/accel quaternion on the DMP at
	 * 200Hz. Cannot be used in combination with DMP_FEATURE_LP_QUAT.
	 * DMP_FEATURE_TAP: Detect taps along the X, Y, and Z axes.
	 * DMP_FEATURE_ANDROID_ORIENT: Google's screen rotation algorithm. Triggers
	 * an event at the four orientations where the screen should rotate.
	 * DMP_FEATURE_GYRO_CAL: Calibrates the gyro data after eight seconds of
	 * no motion.
	 * DMP_FEATURE_SEND_RAW_ACCEL: Add raw accelerometer data to the FIFO.
	 * DMP_FEATURE_SEND_RAW_GYRO: Add raw gyro data to the FIFO.
	 * DMP_FEATURE_SEND_CAL_GYRO: Add calibrated gyro data to the FIFO. Cannot
	 * be used in combination with DMP_FEATURE_SEND_RAW_GYRO.
	 */

	//
	// dmp_load_motion_driver_firmvare
	//
	if (!pMpu6050->dmp_load_motion_driver_firmware())
		DEBUG_PRINT("dmp_load_motion_driver_firmware complete ......\n");
	else
		DEBUG_PRINT("dmp_load_motion_driver_firmware come across error ......\n");

	//
	// dmp_set_orientation
	//
	if (!pMpu6050->dmp_set_orientation(IMU_inv_orientation_matrix_to_scalar(gyro_orientation)))
		DEBUG_PRINT("dmp_set_orientation complete ......\n");
	else
		DEBUG_PRINT("dmp_set_orientation come across error ......\n");

	//
	// dmp_enable_feature
	//
	if (!pMpu6050->dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
	DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
	DMP_FEATURE_GYRO_CAL))
		DEBUG_PRINT("dmp_enable_feature complete ......\n");
	else
		DEBUG_PRINT("dmp_enable_feature come across error ......\n");

	//
	// dmp_set_fifo_rate
	//
	if (!pMpu6050->dmp_set_fifo_rate(DEFAULT_MPU_HZ))
		DEBUG_PRINT("dmp_set_fifo_rate complete ......\n");
	else
		DEBUG_PRINT("dmp_set_fifo_rate come across error ......\n");

	//
	// mpu self test
	//
	IMU_run_self_test(pMpu6050);
	if (!pMpu6050->mpu_set_dmp_state(1))
		DEBUG_PRINT("mpu_set_dmp_state complete ......\n");
	else
		DEBUG_PRINT("mpu_set_dmp_state come across error ......\n");

	DEBUG_PRINT("\nDMP initialization completed!\n\n");
}

unsigned short IMU_inv_row_2_scale(const signed char *row)
{
	unsigned short b;

	if (row[0] > 0)
		b = 0;
	else if (row[0] < 0)
		b = 4;
	else if (row[1] > 0)
		b = 1;
	else if (row[1] < 0)
		b = 5;
	else if (row[2] > 0)
		b = 2;
	else if (row[2] < 0)
		b = 6;
	else
		b = 7;      // error
	return b;
}

unsigned short IMU_inv_orientation_matrix_to_scalar(const signed char *mtx)
{
	unsigned short scalar;

	/*
	 XYZ  010_001_000 Identity Matrix
	 XZY  001_010_000
	 YXZ  010_000_001
	 YZX  000_010_001
	 ZXY  001_000_010
	 ZYX  000_001_010
	 */

	scalar = IMU_inv_row_2_scale(mtx);
	scalar |= IMU_inv_row_2_scale(mtx + 3) << 3;
	scalar |= IMU_inv_row_2_scale(mtx + 6) << 6;

	return scalar;
}
