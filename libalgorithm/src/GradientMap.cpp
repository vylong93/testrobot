/*
 * GradientMap.cpp
 *
 *  Created on: May 13, 2015
 *      Author: VyLong
 */

#include "libalgorithm/inc/GradientMap.h"

GradientMap::GradientMap(void)
{
	this->reset();
}

void GradientMap::reset(void)
{
	/*  Reset Image
		.  .  .  .  .  .  .
		.  x  x  .  x  x  .
		.  .  x  x  x  .  .
		.  .  .  x  .  .  .
		.  .  .  .  .  .  .
	*/

	if(pGradientMap != 0)
		delete[] pGradientMap;

	pGradientMap = new int8_t[Height * Width]; /*{
		-2, -3, -4, -5, -6,  -7, -8,
		-1,  0,  1, -6,  5,   6, -9,
		-2, -3,  2,  3,  4, -11, -10,
		-3, -4, -5,  4, -9, -10, -11,
		-4, -5, -6, -7, -8,  -9, -10 }; */

	if (pGradientMap == 0)
		return;

	Width = 7;
	Height = 5;
	OffsetWidth = -1;
	OffsetHeight = -1;
}

bool GradientMap::modifyGradientMap(uint32_t ui32Height, uint32_t ui32Width, int8_t* pi8NewMap, int8_t i8OffsetHeight, int8_t i8OffsetWidth)
{
	if (pGradientMap != 0)
		delete[] pGradientMap;

	// Allocate new memory zone
	uint64_t ui64NewMapSize = ui32Height * ui32Width;
	pGradientMap = new int8_t[ui64NewMapSize];
	if(pGradientMap == 0)
		return false;

	// Get new gradient map
	int i;
	for(i = 0; i < ui64NewMapSize; i++)
		pGradientMap[i] = pi8NewMap[i];

	Width = ui32Width;
	Height = ui32Height;
	OffsetWidth = i8OffsetWidth;
	OffsetHeight = i8OffsetHeight;
	return true;
}

Vector2<float> GradientMap::coordinateOfTheCenterGradientPixelOfRobotLocation(Vector2<float> vectLocation)
{
	Vector2<int> index = convertRobotCoordinateToGradientMapIndex(vectLocation);

	return convertGradientMapIndexToCoordinate(index);
}

int8_t GradientMap::valueOf(Vector2<float> vectLocation)
{
	Vector2<int> index = convertRobotCoordinateToGradientMapIndex(vectLocation);

	return getValueInMap(index);
}

Vector2<int> GradientMap::convertRobotCoordinateToGradientMapIndex(Vector2<float> vectLocation)
{
	Vector2<int> index;

	//TODO: Robot(x, y) -> gm(y - offsetY, x - offsetX);
	index.x = vectLocation.y - OffsetHeight + 0.5f;
	index.y = vectLocation.x - OffsetWidth + 0.5f;

	index.x = (index.x < 0) ? (index.x - 1) : (index.x);
	index.y = (index.y < 0) ? (index.y - 1) : (index.y);

	return index;
}

Vector2<float> GradientMap::convertGradientMapIndexToCoordinate(Vector2<int> index)
{
	Vector2<float> coordinate;

	//TODO: Robot(x, y) -> gm(y - offsetY, x - offsetX);
	coordinate.x = index.y + OffsetHeight;
	coordinate.y = index.x + OffsetWidth;

	return coordinate;
}

int8_t GradientMap::getValueInMap(Vector2<int> index)
{
	// Case 1: require index is inside the gradient map
	if (index.x >= 0 && index.x < Height && index.y >= 0 && index.y < Width)
		return pGradientMap[index.x * Width + index.y]; // ppGradientMap [x, y];

	// Case 2: require index is outside the gradient map
	Vector2<int> baseIndex;
	if (index.x < 0) {
		if(index.y < 0){
			//Debug.Log("Outside zone 7");
			baseIndex.y = 0;
			baseIndex.x = 0;
		} else if (index.y < Width){
			//Debug.Log("Outside zone 6");
			baseIndex.y = index.y;
			baseIndex.x = 0;
		} else { // y >= WIDTH
			//Debug.Log("Outside zone 5");
			baseIndex.y = Width - 1;
			baseIndex.x = 0;
		}
	} else if (index.x < Height) {
		if(index.y < 0){
			//Debug.Log("Outside zone 8");
			baseIndex.y = 0;
			baseIndex.x = index.x;
		} else { // y >= WIDTH
			//Debug.Log("Outside zone 4");
			baseIndex.y = Width - 1;
			baseIndex.x = index.x;
		}
	} else { // x >= HEIGHT
		if(index.y < 0){
			//Debug.Log("Outside zone 1");
			baseIndex.y = 0;
			baseIndex.x = Height - 1;
		} else if (index.y < Width){
			//Debug.Log("Outside zone 2");
			baseIndex.y = index.y;
			baseIndex.x = Height - 1;
		} else { // y >= WIDTH
			//Debug.Log("Outside zone 3");
			baseIndex.y = Width - 1;
			baseIndex.x = Height - 1;
		}
	}

	int8_t value = pGradientMap[baseIndex.x * Width + baseIndex.y]
				- (int)fabsf(index.x) - (int)fabsf(index.y)
				+ (int)fabsf(baseIndex.x + (int)fabsf(baseIndex.y));

	return value;
}
