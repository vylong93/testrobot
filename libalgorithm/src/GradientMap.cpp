/*
 * GradientMap.cpp
 *
 *  Created on: May 13, 2015
 *      Author: VyLong
 */

#include "libalgorithm/inc/GradientMap.h"
#include "libcustom/inc/custom_uart_debug.h"

GradientMap::GradientMap(void)
{
	pImage = 0;
	pGradientMap = 0;
	Height = Width = 0;
	RowOfStartShapePixel = 0;
	ColumnOfStartShapePixel = 0;
	this->reset();
}

GradientMap::~GradientMap(void) {
	if(pImage != NULL)
		delete[] pImage;

	if (pGradientMap != NULL)
		delete[] pGradientMap;
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

	if (pGradientMap != 0)
		delete[] pGradientMap;

	Width = 7;
	Height = 5;
	RowOfStartShapePixel = 1;
	ColumnOfStartShapePixel = 1;
	pGradientMap = new GradientMapPixel_t[Height * Width]; /*{
	 -2, -1, -2, -3, -4,  -5, -6,
	 -3,  0,  1, -4,  5,   6, -7,
	 -4, -5,  2,  3,  4,  -9, -8,
	 -5, -6, -7,  4, -11, -10, -9,
	 -6, -7, -8, -9, -10, -11, -10 }; */

	if (pGradientMap == 0)
		return;

	//TODO: test only =============================
	int8_t pMap[] = {
			 -2, -1, -2, -3, -4,  -5, -6,
			 -3,  0,  1, -4,  5,   6, -7,
			 -4, -5,  2,  3,  4,  -9, -8,
			 -5, -6, -7,  4, -11, -10, -9,
			 -6, -7, -8, -9, -10, -11, -10  };

	uint64_t i;
	for (i = 0; i < Height * Width; i++)
		pGradientMap[i] = (GradientMapPixel_t) pMap[i];
}

bool GradientMap::modifyGradientMap(int8_t* pi8Image, uint32_t ui32Height, uint32_t ui32Width)
{
	if (pGradientMap != 0)
	{
		delete[] pGradientMap;
		pGradientMap = NULL;
	}

	if (pImage != 0)
	{
		delete[] pImage;
		pGradientMap = NULL;
	}

	// Allocate new memory spaces
	uint64_t ui64NewMapSize = ui32Height * ui32Width;
	pGradientMap = new GradientMapPixel_t[ui64NewMapSize];
	if (pGradientMap == 0)
		return false;

	uint32_t i;

	// Update New GradientMap -------------------------------------
	for (i = 0; i < ui64NewMapSize; i++)
		pGradientMap[i] = 0;

	// Get new image
	pImage = pi8Image;

	Width = ui32Width;
	Height = ui32Height;
	//-------------------------------------------------------------

	Vector2<uint32_t> startPointShapeSegment;
	Vector2<uint32_t> startPointExternalSegment;
	if (!searchTheStartIndexOfSegments(startPointShapeSegment, startPointExternalSegment))
		return false;

	RowOfStartShapePixel = startPointShapeSegment.x;
	ColumnOfStartShapePixel = startPointShapeSegment.y;

	// Clone Map ----------------------------------------------------
	GradientMap segment;
	segment.Height = Height;
	segment.Width = Width;

	// Allocate new memory spaces for clone segment
	segment.pGradientMap = new GradientMapPixel_t[ui64NewMapSize];
	segment.pImage = new int8_t[ui64NewMapSize];
	if (segment.pGradientMap == 0 || segment.pImage == 0)
		return false;

	// Initialize GradientMap for clone segment ----------------------
	for (i = 0; i < ui64NewMapSize; i++)
		segment.pGradientMap[i] = SEGMENT_PIXEL_MASK_INIT;
	//----------------------------------------------------------------


	// Shape Segment: create a clone segment for isolating process ---------------------------
	for (i = 0; i < ui64NewMapSize; i++) // Initialize clone image: only two segment allow
		segment.pImage[i] = (pImage[i] == SHAPE_PIXEL) ? (ACTIVE_SEGMENT) : (DEACTIVE_SEGMENT);

	calculateTheManhattanDistanceOfSegment(startPointShapeSegment, segment, SHAPE_PIXEL);

	for (i = 0; i < ui64NewMapSize; i++) // Apply result to global GradientMap
		pGradientMap[i] += (segment.pGradientMap[i] * segment.pImage[i]);
	//----------------------------------------------------------------------------------------


	// ReInitialize GradientMap for clone segment--------------------------------------------
	for (i = 0; i < ui64NewMapSize; i++)
		segment.pGradientMap[i] = SEGMENT_PIXEL_MASK_INIT;
	// --------------------------------------------------------------------------------------

	for (i = 0; i < ui64NewMapSize; i++) // Initialize clone image: only two segment allow
		segment.pImage[i] = (pImage[i] == (int8_t)(EXTERNAL_PIXEL)) ? (ACTIVE_SEGMENT) : (DEACTIVE_SEGMENT);

	calculateTheManhattanDistanceOfSegment(startPointExternalSegment, segment, EXTERNAL_PIXEL);

	for (i = 0; i < ui64NewMapSize; i++) // Apply result to global GradientMap
		pGradientMap[i] += ((- segment.pGradientMap[i] - 1) * segment.pImage[i]);


	// Clean up ---------------------------
	delete[] segment.pGradientMap;
	segment.pGradientMap = NULL;

	delete[] segment.pImage;
	segment.pImage = NULL;
	// ------------------------------------

	return true;
}

bool GradientMap::searchTheStartIndexOfSegments(Vector2<uint32_t>& shapeStartPoint, Vector2<uint32_t>& externalStartPoint)
{
	// Warning: this function do not scan the first and the last line of the image
	// because the start pixel of the shape segment and the external segment
	// should not be allow to located in this two line.
	uint32_t ui32EndIndex = Height * Width - Width;

	for (uint32_t i = Width; i < ui32EndIndex; i++) {
		if (pImage[i] == SHAPE_PIXEL) {
			// get the shape segment start point
			shapeStartPoint.x = i / Width; // Row
			shapeStartPoint.y = i % Width; // Column

			// External Segment start point is the upper pixel of the shape segment start point
			externalStartPoint.x = shapeStartPoint.x - 1;
			externalStartPoint.y = shapeStartPoint.y;

			return true;
		}
	}
	return false;
}

void GradientMap::calculateTheManhattanDistanceOfSegment(Vector2<uint32_t>& startPoint, GradientMap& Segment, int8_t i8ImageValue)
{
	//IDEAD: calculate the shape bolder first (smaller distance have high priority)
	// From start pixel, follow bolder in both direction (left to right and top to bottom)
	// in the same step, if these two scanning pointer collision, choose to the smaller value
	// and keep go on until (approach 1: go back to original. approach 2: collision continue repeat over 5 times.

	// Simple approach -> scan and scan
	Segment.pGradientMap[startPoint.x * Segment.Width + startPoint.y] = 0; // Initialize starting point

	bool bIsAtBolderTop;
	bool bIsAtBolderBottom;
	bool bIsAtBolderLeft;
	bool bIsAtBolderRight;
	int32_t index[4];	// up, down, left , right

	uint32_t startRow = 0;
	uint32_t endRow = Segment.Height;

	uint32_t nextStartRow;
	uint32_t nextEndRow;

	uint32_t loopCounter = 0;
	uint32_t pixelScanTimes = 0;
	bool bIsNeedToLoopAgain = false;
	bool bIsNeedToFindNewStartPoint = false;
	bool bIsCalculatedNewManhattanDistance;
	while(true)
	{
		loopCounter++;
		bIsCalculatedNewManhattanDistance = false;
		for(uint32_t row = startRow; row < endRow; row++)
		{
			for(uint32_t col = 0; col < Segment.Width; col++)
			{
				pixelScanTimes++;

				uint32_t indexSelf = row * Segment.Width + col;
				if (Segment.pImage[indexSelf] == ACTIVE_SEGMENT && Segment.pGradientMap[indexSelf] == SEGMENT_PIXEL_MASK_INIT)
				{
					if(bIsNeedToFindNewStartPoint)
					{
						bIsNeedToFindNewStartPoint = false;
						//TODO: find new start point and set pImage to i32ImageValue
						pImage[indexSelf] = i8ImageValue;
						Segment.pGradientMap[indexSelf] = 0;
					}
					else
					{
						bIsAtBolderTop = (row == 0);
						bIsAtBolderBottom = (row == (Segment.Height - 1));
						bIsAtBolderLeft = (col == 0);
						bIsAtBolderRight = (col == (Segment.Width - 1));

						// get index
						index[0] = (bIsAtBolderTop) ? (SEGMENT_PIXEL_INVALID) : (((int32_t)row - 1) * (int32_t)Segment.Width + (int32_t)col); // up
						index[1] = (bIsAtBolderBottom) ? (SEGMENT_PIXEL_INVALID) : (((int32_t)row + 1) * (int32_t)Segment.Width + (int32_t)col); // down
						index[2] = (bIsAtBolderLeft) ? (SEGMENT_PIXEL_INVALID) : ((int32_t)indexSelf - 1); // left
						index[3] = (bIsAtBolderRight) ? (SEGMENT_PIXEL_INVALID) : ((int32_t)indexSelf + 1); // right

						// try to get manhattan distance
						GradientMapPixel_t manhattanDistance;
						GradientMapPixel_t minManhattan;
						bool bFoundManhattan = false;
						for(uint32_t k = 0; k < 4; k++)
						{
							int32_t indexK = index[k];
							if (indexK != SEGMENT_PIXEL_INVALID)
							{
								manhattanDistance = Segment.pGradientMap[indexK];
								if (manhattanDistance != SEGMENT_PIXEL_MASK_INIT)
								{
									if(!bFoundManhattan)
									{
										bFoundManhattan = true;
										minManhattan = manhattanDistance;
									}
									else
									{
										if (manhattanDistance < minManhattan)
											minManhattan = manhattanDistance;
									}
								}
							}
						}

						if (bFoundManhattan)
						{
							Segment.pGradientMap[indexSelf] = minManhattan + 1;
							if(i8ImageValue < 0)
								pImage[indexSelf] = i8ImageValue;
							bIsCalculatedNewManhattanDistance = true;
						}
						else
						{
							if(!bIsNeedToLoopAgain)
							{
								bIsNeedToLoopAgain = true;
								nextStartRow = row;
							}
							nextEndRow = row;
						}
					}
				}
			}
		}

		if(bIsNeedToLoopAgain)
		{
			bIsNeedToLoopAgain = false;
			startRow = nextStartRow;
			endRow = nextEndRow + 1;

			if(!bIsCalculatedNewManhattanDistance)
			{
				// completed external but found trapped segment
				bIsNeedToFindNewStartPoint = true;
				if(i8ImageValue <= 0)
					i8ImageValue--;
			}
		}
		else
			break;
	};

	DEBUG_PRINTS3("Total loop count = %d, pixel scan = %d [%3.2f %%]\n", loopCounter, pixelScanTimes, pixelScanTimes * 100.0f / (Segment.Height * Segment.Width));
}

void GradientMap::coordinateOfTheCenterGradientPixelOfRobotLocation(
		Vector2<float>& rvectLocation, Vector2<float>& rvectCenterLocation) {
	Vector2<int> vectIndex;
	convertRobotCoordinateToGradientMapIndex(rvectLocation, vectIndex);

	convertGradientMapIndexToCoordinate(vectIndex, rvectCenterLocation);
}

int32_t GradientMap::valueOf(Vector2<float>& rvectLocation) {
	Vector2<int> vectIndex;
	convertRobotCoordinateToGradientMapIndex(rvectLocation, vectIndex);

	return getValueInMap(vectIndex);
}

int32_t GradientMap::valueOf(float x, float y) {
	Vector2<float> vectLocation(x, y);
	Vector2<int> vectIndex;
	convertRobotCoordinateToGradientMapIndex(vectLocation, vectIndex);

	return getValueInMap(vectIndex);
}



void GradientMap::convertRobotCoordinateToGradientMapIndex(
		Vector2<float>& rvectLocation, Vector2<int>& rvectIndex) {
//  /* Attemp 1: */
//	index.x = (int)(vectLocation.x / PIXEL_HALFSIZE_IN_CM);
//  if (index.x % 2 != 0)
//	{
//		if (index.x > 0)
//			index.x += 1;
//		else
//			index.x -= 1;
//	}
//	index.x = index.x / 2 - RowOfStartShapePixel;

//	/* Attemp 2: */
//	index.y = (int)(vectLocation.y / PIXEL_HALFSIZE_IN_CM);
//	index.y = (index.y % 2 == 0) ? (index.y) : ((index.y > 0) ? (index.y + 1) : (index.y - 1));
//	index.y = index.y / 2 - ColumnOfStartShapePixel;

	int signX = (rvectLocation.x < 0) ? (-1) : (1);
	rvectIndex.x = (int) (fabsf(rvectLocation.x) / PIXEL_HALFSIZE_IN_CM);
	rvectIndex.x = signX * ((rvectIndex.x + (rvectIndex.x % 2)) / 2)
			+ RowOfStartShapePixel;

	int signY = (rvectLocation.y < 0) ? (-1) : (1);
	rvectIndex.y = (int) (fabsf(rvectLocation.y) / PIXEL_HALFSIZE_IN_CM);
	rvectIndex.y = signY * ((rvectIndex.y + (rvectIndex.y % 2)) / 2)
			+ ColumnOfStartShapePixel;
}

void GradientMap::convertGradientMapIndexToCoordinate(Vector2<int>& rvectIndex,
		Vector2<float>& rvectCoordinate) {
	rvectCoordinate.x = (rvectIndex.x - RowOfStartShapePixel) * PIXEL_SIZE_IN_CM;
	rvectCoordinate.y = (rvectIndex.y - ColumnOfStartShapePixel) * PIXEL_SIZE_IN_CM;
}

int32_t GradientMap::getValueInMap(Vector2<int32_t>& rvectIndex) {
	// Case 1: require index is inside the gradient map
	if (rvectIndex.x >= 0 && rvectIndex.x < (int32_t) Height
			&& rvectIndex.y >= 0 && rvectIndex.y < (int32_t) Width)
		return pGradientMap[rvectIndex.x * Width + rvectIndex.y]; // ppGradientMap [x, y];

	// Case 2: require index is outside the gradient map
	Vector2<int> baseIndex;
	if (rvectIndex.x < 0) {
		if (rvectIndex.y < 0) {
			DEBUG_PRINT("Outside zone 7\n");
			baseIndex.y = 0;
			baseIndex.x = 0;
		} else if (rvectIndex.y < (int32_t) Width) {
			DEBUG_PRINT("Outside zone 6\n");
			baseIndex.y = rvectIndex.y;
			baseIndex.x = 0;
		} else { // y >= WIDTH
			DEBUG_PRINT("Outside zone 5\n");
			baseIndex.y = (int32_t) Width - 1;
			baseIndex.x = 0;
		}
	} else if (rvectIndex.x < (int32_t) Height) {
		if (rvectIndex.y < 0) {
			DEBUG_PRINT("Outside zone 8\n");
			baseIndex.y = 0;
			baseIndex.x = rvectIndex.x;
		} else { // y >= WIDTH
			DEBUG_PRINT("Outside zone 4\n");
			baseIndex.y = (int32_t) Width - 1;
			baseIndex.x = rvectIndex.x;
		}
	} else { // x >= HEIGHT
		if (rvectIndex.y < 0) {
			DEBUG_PRINT("Outside zone 1\n");
			baseIndex.y = 0;
			baseIndex.x = Height - 1;
		} else if (rvectIndex.y < (int32_t) Width) {
			DEBUG_PRINT("Outside zone 2\n");
			baseIndex.y = rvectIndex.y;
			baseIndex.x = Height - 1;
		} else { // y >= WIDTH
			DEBUG_PRINT("Outside zone 3\n");
			baseIndex.y = Width - 1;
			baseIndex.x = Height - 1;
		}
	}

	int32_t value = pGradientMap[baseIndex.x * Width + baseIndex.y]
			- (int) fabsf(rvectIndex.x) - (int) fabsf(rvectIndex.y)
			+ (int) fabsf(baseIndex.x + (int) fabsf(baseIndex.y));

	return value;
}
