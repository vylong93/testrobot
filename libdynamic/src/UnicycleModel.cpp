/*
 * UnicycleModel.cpp
 *
 *  Created on: Apr 26, 2015
 *      Author: VyLong
 */

#include "libdynamic/inc/UnicycleModel.h"
#include "libdynamic/inc/DifferentialDriveConstant.h"

UnicycleModel::UnicycleModel() { v = 0; w = 0; }

UnicycleModel::UnicycleModel(float iv, float iw) { v = iv; w = iw; }

void UnicycleModel::toDifferentialDrive(float &vel_r, float &vel_l)
{
	vel_r = (2 * v + w * WHEEL_BASE_LENGTH) / (2 * WHEEL_RADIUS);
	vel_l = (2 * v - w * WHEEL_BASE_LENGTH) / (2 * WHEEL_RADIUS);
}
