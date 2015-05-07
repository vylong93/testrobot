/*
 * ForwardController.cpp
 *
 *  Created on: May 4, 2015
 *      Author: VyLong
 */

#include "libcontroller/inc/ForwardController.h"
#include <math.h>

ForwardController::ForwardController() {
	E = 0;
	e_old = 0;
	RunStep = 0;
}

void ForwardController::execute(RobotIdentity_t input, UnicycleModel &output)
{
	// Compute the v, w that will get you to the goal
	output.v = Speed;

	// 1. Calculate the heading error.
	float e_k = input.theta - TrackTheta;

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

	RunStep--;
}

void ForwardController::reset()
{
	E = 0;
	e_old = 0;
	RunStep = 0;
}






