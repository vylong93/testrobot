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

#define PIXEL_SIZE_IN_CM		20
#define PIXEL_HALFSIZE_IN_CM	10

#define SHAPE_PIXEL		1
#define EXTERNAL_PIXEL 	0
//#define TRAPPED1_PIXEL	-1
//#define TRAPPED2_PIXEL	-2
//#define TRAPPED3_PIXEL	-3
//#define TRAPPED4_PIXEL	-4
//#define TRAPPED5_PIXEL	-5

#define ACTIVE_SEGMENT		1
#define DEACTIVE_SEGMENT	0

#define SEGMENT_PIXEL_MASK_INIT 			-15
#define SEGMENT_PIXEL_INVALID				-2
#define SEGMENT_PIXEL_MASK_NON_MANHATTAN	-1
#define SEGMENT_PIXEL_MASK_NON_VECTOR		-1

#define START_PIXEL_VALUE_OF_NON_SHAPE_SEGMENT	-1

typedef enum tag_SegmentType
{
	SEGMENT_SHAPE,
	SEGMENT_EXTERNAL,
	SEGMENT_TRAPPED
} e_SegmentType;

class GradientMap {
public: //TODO: change to private members
	typedef int16_t GradientMapPixel_t;

	int8_t* pImage;
	GradientMapPixel_t* pGradientMap;
	uint32_t Height;
	uint32_t Width;
	int32_t RowOfStartShapePixel;
	int32_t ColumnOfStartShapePixel;

	// Constructors
	GradientMap(void);
	~GradientMap(void);

	// Methods
	void reset(void);
	e_SegmentType getSegmentType(Vector2<float>& rvectLocation);
	bool modifyGradientMap(int8_t* pi8Image, uint32_t ui32Height, uint32_t ui32Width);


	void coordinateOfTheCenterGradientPixelOfRobotLocation(Vector2<float>& rvectLocation, Vector2<float>& rvectCenterLocation);
	int32_t valueOf(Vector2<float>& rvectLocation);
	int32_t valueOf(float x, float y);

	//private:
	bool searchTheStartIndexOfSegments(Vector2<uint32_t>& shapeStartPoint, Vector2<uint32_t>& externalStartPoint);
	void calculateTheManhattanDistanceOfSegment(Vector2<uint32_t>& startPoint, GradientMap& Segment, int8_t i8ImageValue);

	void convertRobotCoordinateToGradientMapIndex(Vector2<float>& rvectLocation, Vector2<int>& rvectIndex);
	void convertGradientMapIndexToCoordinate(Vector2<int>& rvectIndex, Vector2<float>& rvectCoordinate);
	int32_t getValueInMap(Vector2<int>& rvectIndex);
};

#endif /* LIBALGORITHM_INC_GRADIENTMAP_H_ */
