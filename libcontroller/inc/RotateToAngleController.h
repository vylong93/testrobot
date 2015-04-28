/*
 * RotateToAngleController.h
 *
 *  Created on: Apr 28, 2015
 *      Author: VyLong
 */

#ifndef LIBCONTROLLER_INC_ROTATETOANGLECONTROLLER_H_
#define LIBCONTROLLER_INC_ROTATETOANGLECONTROLLER_H_

#include "libstorage/inc/RobotIdentity.h"
#include "libdynamic/inc/UnicycleModel.h"

class RotateToAngleController {
private:
	// Memory banks
	float E;
	float e_old;

public:
	float StartTheta;
	float EndTheta;

	float kP; // 35
	float kI; // 0.25
	float kD; // 0.0

	RotateToAngleController();
	void execute(RobotIdentity_t input, UnicycleModel &output);
	void reset();
};

#endif /* LIBCONTROLLER_INC_ROTATETOANGLECONTROLLER_H_ */
