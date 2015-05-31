/*
 * GradientUnit.h
 *
 *  Created on: May 31, 2015
 *      Author: VyLong
 */

#ifndef LIBCONTROLLER_INC_GRADIENTUNIT_H_
#define LIBCONTROLLER_INC_GRADIENTUNIT_H_

#include <stdint.h>
#include <stdbool.h>
#include "libmath/inc/Vector2.h"

class GradientUnit
{
public:
	int8_t Value;
	Vector2<float>* pPosition;

	GradientUnit();
	GradientUnit(Vector2<float>* pPoint, int8_t value);
	~GradientUnit();

	void setContent(Vector2<float>* pPoint, int8_t value);
};



#endif /* LIBCONTROLLER_INC_GRADIENTUNIT_H_ */
