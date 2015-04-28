/*
 * RotateToAngleController.cpp
 *
 *  Created on: Apr 28, 2015
 *      Author: VyLong
 */

#include "libcontroller/inc/RotateToAngleController.h"
#include <math.h>

RotateToAngleController::RotateToAngleController() {
	E = 0;
	e_old = 0;
}

void RotateToAngleController::execute(RobotIdentity_t input, UnicycleModel &output)
{
	// Compute the v, w that will get you to the goal
	output.v = 0;

	// 1. Calculate the heading error.
	float e_k = input.theta - EndTheta;

	// Hint: Use ATAN2 to make sure this stays in [-pi, pi].
	e_k = atan2f(sinf(e_k), cosf(e_k));

	// calculate the error dynamic
	float e_dot = e_k - e_old;

	// integral the error
	E = E + e_k;

	// compute the control input
	output.w = kP*e_k + kI*E + kD*e_dot;

	// save 'e' for the next loop
	e_old = e_k;
}

void RotateToAngleController::reset()
{
	E = 0;
	e_old = 0;
}


