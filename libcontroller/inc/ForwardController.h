/*
 * ForwardController.h
 *
 *  Created on: May 4, 2015
 *      Author: VyLong
 */

#ifndef LIBCONTROLLER_INC_FORWARDCONTROLLER_H_
#define LIBCONTROLLER_INC_FORWARDCONTROLLER_H_

#include "libstorage/inc/RobotIdentity.h"
#include "libdynamic/inc/UnicycleModel.h"

class ForwardController {
private:
	// Memory banks
	float E;
	float e_old;

public:
	unsigned int RunStep;
	char Speed;
	float TrackTheta;

	float kP; // 35
	float kI; // 0.25
	float kD; // 0.0

	ForwardController();
	void execute(RobotIdentity_t input, UnicycleModel &output);
	void reset();
};


#endif /* LIBCONTROLLER_INC_FORWARDCONTROLLER_H_ */
