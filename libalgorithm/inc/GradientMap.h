/*
 * GradientMap.h
 *
 *  Created on: May 13, 2015
 *      Author: VyLong
 */

#ifndef LIBALGORITHM_INC_GRADIENTMAP_H_
#define LIBALGORITHM_INC_GRADIENTMAP_H_

#include <stdint.h>
#include <stdbool.h>
#include "libmath/inc/Vector2.h"

// Two Dimention Array attempts

// 	Attempt 1: Normal way. This is not exactly a light weight solution
//	int** ary = new int[sizeY][sizeX]
//	should be:
//
//	int **ary = new int*[sizeY];
//	for(int i = 0; i < sizeY; ++i) {
//		ary[i] = new int[sizeX];
//	}
//
//	and then clean up would be:
//
//	for(int i = 0; i < sizeY; ++i) {
//		delete [] ary[i];
//	}
//	delete [] ary;

//	Attempt 2: This is a alternative approach which use one large block of memory
//	int *ary = new int[sizeX*sizeY];
//	ary[i][j] is then rewritten as ary[i*sizeY+j]

class GradientMap {
public: //TODO: change to private members
	int8_t* pGradientMap;
	uint32_t Height;
	uint32_t Width;
	int8_t OffsetWidth;
	int8_t OffsetHeight;

	// Constructor
	GradientMap(void);

	// Methods
	void reset(void);
	bool modifyGradientMap(uint32_t ui32Height, uint32_t ui32Width, int8_t* pi8NewMap, int8_t i8OffsetHeight, int8_t i8OffsetWidth);

	Vector2<float> coordinateOfTheCenterGradientPixelOfRobotLocation(Vector2<float> vectLocation);
	int8_t valueOf(Vector2<float> vectLocation);
	Vector2<int> convertRobotCoordinateToGradientMapIndex(Vector2<float> vectLocation);
	Vector2<float> convertGradientMapIndexToCoordinate(Vector2<int> index);
	int8_t getValueInMap(Vector2<int> index);
};

#endif /* LIBALGORITHM_INC_GRADIENTMAP_H_ */
