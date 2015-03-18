/*
 * GradientDescentNode.h
 *
 *  Created on: Mar 18, 2015
 *      Author: VyLong
 */

#ifndef LIBALGORITHM_INC_GRADIENTDESCENTNODE_H_
#define LIBALGORITHM_INC_GRADIENTDESCENTNODE_H_


#include "libstorage/inc/RobotLocation.h"
#include "libmath/inc/Vector2.h"

#define STEP_SIZE		0.2f
#define STOP_CONDITION	0.3f

class GradientDescentNode {

private:
	float StepSize;
	float StopCondition;

	RobotLocation* pRobot;
	int oneHopIndex;

	Vector2<float> EstimatePosNew;
	Vector2<float> EstimatePosOld;
	Vector2<float> GradienNew;
	Vector2<float> GradienOld;

public:
	bool isGradientSearchStop;

	// Functions
	void init(RobotLocation* pTarget, float fStepSize = STEP_SIZE, float fStopCondition = STOP_CONDITION);
	void run(void);
};


#endif /* LIBALGORITHM_INC_GRADIENTDESCENTNODE_H_ */
