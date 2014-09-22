/*
 * TDOA.h
 *
 *  Created on: Sep 6, 2014
 *      Author: MasterE
 */

#ifndef TDOA_H_
#define TDOA_H_

#include "librobot/inc/MainBoardDriver.h"

#define FILTER_ORDER                    34
#define BLOCK_SIZE                      10
#define START_SAMPLES_POSTITION         40
#define NUM_BLOCKS                      (NUMBER_OF_SAMPLE / BLOCK_SIZE)
#define NUM_DATAS                       (NUMBER_OF_SAMPLE - START_SAMPLES_POSTITION)
#define MAX_THRESHOLD					50
#define MAX_SAMPLE_POSITION				215

inline void TDOA_initFilters(float32_t* FilterCoeffs);

void TDOA_run(void);

float32_t TDOA_getDistances(float32_t *myData, float32_t *peakEnvelope, float32_t *maxEnvelope);

void TDOA_find3LocalPeaks(float32_t *myData, float32_t *LocalPeaksStoragePointer);

uint32_t TDOA_reachBottom(float32_t *myData, uint32_t const PeakPosition,
                uint32_t const PointerIncreaseNumber);

uint32_t TDOA_reachPeak(float32_t *myData, uint32_t const PeakPosition,
                uint32_t const PointerIncreaseNumber);

void TDOA_interPeak(float32_t* PositionsArray, float32_t* ValuesArray, float32_t UserPosition,
                float32_t UserMaxValue, float32_t const step, float32_t* ReturnPosition,
                float32_t* ReturnValue);

float32_t TDOA_larange(float32_t *PositionsArray, float32_t *ValuesArray, float32_t interpolatePoint);


#endif /* TDOA_H_ */
