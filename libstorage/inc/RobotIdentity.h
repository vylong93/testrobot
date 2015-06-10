/*
 * RobotIdentity.h
 *
 *  Created on: Mar 21, 2015
 *      Author: VyLong
 */

#ifndef ROBOTIDENTITY_H_
#define ROBOTIDENTITY_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

typedef enum tag_Locomotion
{
	LOCOMOTION_SAME = 0,
	LOCOMOTION_DIFFERENT = 1,
	LOCOMOTION_INVALID = 2,
} e_Locomotion;

typedef struct tagRobotIdentity {
	uint32_t Self_ID;
	uint32_t Origin_ID;
	uint32_t RotationHop_ID;
	uint8_t Self_NeighborsCount;
	uint8_t Origin_NeighborsCount;
	uint8_t Origin_Hopth;
	float x;
	float y;
	float theta;
	float RotationHop_x;
	float RotationHop_y;
	bool ValidOrientation;
	bool ValidLocation;
	e_Locomotion Locomotion;
	bool IsMoving;
	bool IsSampling;
} RobotIdentity_t;

#endif /* ROBOTIDENTITY_H_ */
