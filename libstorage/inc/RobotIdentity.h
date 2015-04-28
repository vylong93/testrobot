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
} RobotIdentity_t;


#endif /* ROBOTIDENTITY_H_ */
