/*
 * DASHController.h
 *
 *  Created on: May 31, 2015
 *      Author: VyLong
 */

#ifndef LIBALGORITHM_INC_DASHCONTROLLER_H_
#define LIBALGORITHM_INC_DASHCONTROLLER_H_

#include "libstorage/inc/RobotIdentity.h"
#include "libalgorithm/inc/GradientMap.h"
#include "libstorage/inc/robot_data.h"
#include "libmath/inc/Vector2.h"

#include "libcontroller/inc/GradientUnit.h"

class DASHController
{
private:
	GradientMap* pGradientMap;
	RobotIdentity_t* pRobotIdentity;

	//TODO: define this function in custom_math
	bool isHaveClearshotToTheGoal(Vector2<float>* pPointCurrent, Vector2<float>* pPointGoal);

	void selectionSortGradientUnitArray(GradientUnit pGu[], int Length);
	void reArangeGradientUnitArray(GradientUnit pGu[], GradientUnit& guRobot);
	void swapGradientUnit(GradientUnit& a, GradientUnit& b);

public:
	DASHController(GradientMap* pGMap, RobotIdentity_t* pRobotId);
	~DASHController();

	void calculateTheNextGoal(Vector2<float>* pPointNextGoal);
	bool isTwoPositionOverlay(Vector2<float>* pPointA, Vector2<float>* pPointB, float errorInCm);
};


#endif /* LIBALGORITHM_INC_DASHCONTROLLER_H_ */
