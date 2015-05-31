/*
 * Controller.h
 *
 *  Created on: Apr 26, 2015
 *      Author: VyLong
 */

#ifndef LIBCONTROLLER_CONTROLLER_H_
#define LIBCONTROLLER_CONTROLLER_H_

#include "libmath/inc/Vector2.h"
#include "libstorage/inc/RobotIdentity.h"
#include "libdynamic/inc/UnicycleModel.h"

#define CONTROLLER_ANGLE_ERROR_DEG			2.0f
#define CONTROLLER_POSITION_ERROR_CM		2.0f
#define CONTROLLER_POSITION_MOVE_MARRGIN_CM	15.0f	// 2*R (5) + 2 * Margin (1.5)

#define CONTROLLER_VALID_GOAL_DISTANCE		1.3f

class Controller
{
public:
	virtual void execute(RobotIdentity_t input, UnicycleModel &output, Vector2<float> goal, float v, float dt);
	virtual void reset();
};

#endif /* LIBCONTROLLER_CONTROLLER_H_ */
