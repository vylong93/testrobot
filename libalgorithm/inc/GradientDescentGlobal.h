/*
 * GradientDescentGlobal.h
 *
 *  Created on: Mar 27, 2015
 *      Author: VyLong
 */

#ifndef GRADIENTDESCENTGLOBAL_H_
#define GRADIENTDESCENTGLOBAL_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "libmath/inc/Vector2.h"

#define GRADIENT_GLOBAL_STEP_SIZE		0.2f
#define GRADIENT_GLOBAL_STOP_CONDITION	0.3f
#define GRADIENT_GLOBAL_STOP_CONDITION_LOCAL_MINIMA	0.3f

void GradientDescent_updateGradient(uint32_t ui32SelfId,
		Vector2<float> vectSelf, Vector2<float>* pvectGradientNew,
		bool bEnableRandomCalculation);

void GradientDescent_updatePosition(Vector2<float>* pvectAverageCoordination,
		Vector2<float>* pvectEstimatePosNew,
		Vector2<float>* pvectEstimatePosOld, Vector2<float>* pvectGradientNew,
		Vector2<float>* pvectGradientOld, float fStepSize);

bool GradientDescent_checkVarianceCondition(Vector2<float> vectEstimatePosNew,
		Vector2<float> vectEstimatePosOld, float fStopCondition);

#endif /* GRADIENTDESCENTGLOBAL_H_ */
