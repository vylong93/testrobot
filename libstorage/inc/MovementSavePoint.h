/*
 * MovementSavePoint.h
 *
 *  Created on: May 27, 2015
 *      Author: VyLong
 */

#ifndef LIBSTORAGE_INC_MOVEMENTSAVEPOINT_H_
#define LIBSTORAGE_INC_MOVEMENTSAVEPOINT_H_

#include "libstorage/inc/RobotIdentity.h"
#include "libmath/inc/Vector2.h"

#define NUMBER_OF_SAVE_POINTS	3

int getSavePointCounter(void);
void pushNewPoint(float x, float y);
bool getLastPoint(int offset, Vector2<float>* pPointOut);
bool calculateLastForwardOrientation(float *pTheta);
bool calculateLastBackwardOrientation(float *pTheta);
e_Locomotion calculateLastLocomotion(Vector2<float> pPoint[]);


#endif /* LIBSTORAGE_INC_MOVEMENTSAVEPOINT_H_ */
