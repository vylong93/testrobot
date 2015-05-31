/*
 * GradientUnit.cpp
 *
 *  Created on: May 31, 2015
 *      Author: VyLong
 */

#include "libcontroller/inc/GradientUnit.h"

GradientUnit::GradientUnit()
{
	this->Value = 0;
	this->pPosition = NULL;
}

GradientUnit::GradientUnit(Vector2<float>* pPoint, int8_t value)
{
	this->Value = value;
	this->pPosition = pPoint;
}

GradientUnit::~GradientUnit()
{}

void GradientUnit::setContent(Vector2<float>* pPoint, int8_t value)
{
	this->Value = value;
	this->pPosition = pPoint;
}
