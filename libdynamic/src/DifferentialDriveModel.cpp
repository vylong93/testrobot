/*
 * DifferentialDriveModel.cpp
 *
 *  Created on: Apr 26, 2015
 *      Author: VyLong
 */

#include "libdynamic/inc/DifferentialDriveModel.h"
#include "libdynamic/inc/DifferentialDriveConstant.h"

DifferentialDriveModel::DifferentialDriveModel() { vel_r = 0; vel_l = 0; }

DifferentialDriveModel::DifferentialDriveModel(float vr, float vl) { vel_r = vr; vel_l = vl; }

void DifferentialDriveModel::toUnicycle(float &v, float &w)
{
	v = WHEEL_RADIUS / 2 * (vel_r + vel_l);
	w = WHEEL_RADIUS / WHEEL_BASE_LENGTH * (vel_r - vel_l);
}
