/*
 * TDOA.h
 *
 *  Created on: Sep 6, 2014
 *      Author: MasterE
 */

#ifndef TDOA_H_
#define TDOA_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "arm_math.h"
#include "librobot/inc/robot_analog.h"

#define DEFAULT_INTERCEPT_VALUE			8.4049f
#define DEFAULT_SLOPE_VALUE				3.169f

#define DISTANCE_BETWEEN_TWO_MICS		5.0
#define DISTANCE_BETWEEN_TWO_MICS_SQR	25.0	// DISTANCE_BETWEEN_TWO_MICS * DISTANCE_BETWEEN_TWO_MICS

#define FILTER_ORDER                    34
#define BLOCK_SIZE                      10
#define START_SAMPLES_POSTITION         32
#define	STATE_BUFFER_SIZE				(BLOCK_SIZE + FILTER_ORDER - 1)
#define NUM_BLOCKS                      (NUMBER_OF_SAMPLE / BLOCK_SIZE)
#define NUM_DATAS                       (NUMBER_OF_SAMPLE - START_SAMPLES_POSTITION)

#define NOISY_THRESHOLD					50
//#define MAX_SAMPLE_POSITION			215

#define SPEAKER_RADIUS_IN_CM		2.0
#define OFFSET_DISTANCE_IN_CM		SPEAKER_RADIUS_IN_CM

void TDOA_setIntercept(float);
float TDOA_getIntercept(void);
void TDOA_setSlope(float);
float TDOA_getSlope(void);

void TDOA_initFilters();

uint16_t TDOA_calculateDistanceFromTwoPeaks(float fPeakEnvelopeA, float fPeakEnvelopeB, float fIntercept, float fSlope);

void TDOA_process(uint16_t* pui16ADCResult, float* pfPeakEnvelope, float* pfMaxEnvelope);
void TDOA_filterSignalsMic(uint16_t* pui16ADCResult);
bool TDOA_isFilteredSignalNoisy(void);

float TDOA_getDistances(float *myData, float *peakEnvelope, float *maxEnvelope);

void TDOA_find3LocalPeaks(float *myData, float *LocalPeaksStoragePointer);

uint32_t TDOA_reachBottom(float *myData, uint32_t const PeakPosition,
                int32_t const PointerIncreaseNumber);

uint32_t TDOA_reachPeak(float *myData, uint32_t const PeakPosition,
                int32_t const PointerIncreaseNumber);

void TDOA_interPeak(float* PositionsArray, float* ValuesArray, float UserPosition,
                float UserMaxValue, float const step, float* ReturnPosition,
                float* ReturnValue);

float TDOA_larange(float *PositionsArray, float *ValuesArray, float interpolatePoint);


#ifdef __cplusplus
}
#endif

#endif /* TDOA_H_ */
