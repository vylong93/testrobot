/*
 * TDOA.c
 *
 *  Created on: Sep 6, 2014
 *      Author: MasterE
 */

#include "librobot/inc/TDOA.h"

extern uint16_t g_pui16ADC0Result[];
extern uint16_t g_pui16ADC1Result[];

extern unsigned char countAdcDMAsStopped;

// Initialize two filter, input and output buffer pointers
float32_t OutputMicA[NUMBER_OF_SAMPLE] =
{ 0 };
float32_t OutputMicB[NUMBER_OF_SAMPLE] =
{ 0 };

float32_t SamplesMicA[NUMBER_OF_SAMPLE] =
{ 0 };
float32_t SamplesMicB[NUMBER_OF_SAMPLE] =
{ 0 };

arm_fir_instance_f32 FilterA;
arm_fir_instance_f32 FilterB;
static float32_t pStateA[BLOCK_SIZE + FILTER_ORDER - 1] =
{ 0 };
static float32_t pStateB[BLOCK_SIZE + FILTER_ORDER - 1] =
{ 0 };

float32_t g_f32PeakEnvelopeA;
float32_t g_f32MaxEnvelopeA;
float32_t g_f32PeakEnvelopeB;
float32_t g_f32MaxEnvelopeB;

void TDOA_initFilters(float32_t* FilterCoeffs)
{
	// Call FIR init function to initialize the instance structure.
	arm_fir_init_f32(&FilterA, FILTER_ORDER, FilterCoeffs, pStateA, BLOCK_SIZE);
	arm_fir_init_f32(&FilterB, FILTER_ORDER, FilterCoeffs, pStateB, BLOCK_SIZE);
}

void TDOA_run(void)
{
//	float32_t OutputMicA[NUMBER_OF_SAMPLE] = { 0 };
//	float32_t OutputMicB[NUMBER_OF_SAMPLE] = { 0 };
//
//	float32_t SamplesMicA[NUMBER_OF_SAMPLE] = { 0 };
//	float32_t SamplesMicB[NUMBER_OF_SAMPLE] = { 0 };

	int i;

	while (countAdcDMAsStopped != 2)
		;

	// Convert g_ui32ADC0/1Result to SamplesMicA/B
	for (i = 0; i < NUMBER_OF_SAMPLE; i++)
	{
		SamplesMicA[i] = g_pui16ADC0Result[i];
		SamplesMicB[i] = g_pui16ADC1Result[i];
	}

	// Filter signals
	for (i = 0; i < NUM_BLOCKS; i++)
	{
		arm_fir_f32(&FilterA, SamplesMicA + (i * BLOCK_SIZE),
				OutputMicA + (i * BLOCK_SIZE),
				BLOCK_SIZE);
		arm_fir_f32(&FilterB, SamplesMicB + (i * BLOCK_SIZE),
				OutputMicB + (i * BLOCK_SIZE),
				BLOCK_SIZE);
	}

	// Drop some invalid samples at the begin output buffer
	for (i = 0; i < NUMBER_OF_SAMPLE; i++)
	{
		OutputMicA[i] = OutputMicA[i + START_SAMPLES_POSTITION];
		OutputMicB[i] = OutputMicB[i + START_SAMPLES_POSTITION];
	}

	TDOA_getDistances(OutputMicA, &g_f32PeakEnvelopeA, &g_f32MaxEnvelopeA);
	TDOA_getDistances(OutputMicB, &g_f32PeakEnvelopeB, &g_f32MaxEnvelopeB);
}

float32_t TDOA_getDistances(float32_t *myData, float32_t *peakEnvelope,
		float32_t *maxEnvelope)
{
	float32_t step = +0.125f;
	float32_t localPeaksPosition[3] =
	{ 0 };
	float32_t localMaxValue[3] =
	{ 0 };

	TDOA_find3LocalPeaks(myData, localPeaksPosition);
	if (localPeaksPosition[0] != 0 && localPeaksPosition[1] != 0
			&& localPeaksPosition[2] != 0)
	{
		float32_t PositionsArray[3] =
		{ 0 };
		float32_t ValuesArray[3] =
		{ 0 };
		int i;
		for (i = 0; i < 3; i++)
		{
			PositionsArray[0] = localPeaksPosition[i] - 1;
			PositionsArray[1] = localPeaksPosition[i];
			PositionsArray[2] = localPeaksPosition[i] + 1;
			ValuesArray[0] = *(myData + (uint32_t) PositionsArray[0]);
			ValuesArray[1] = *(myData + (uint32_t) PositionsArray[1]);
			ValuesArray[2] = *(myData + (uint32_t) PositionsArray[2]);
			localMaxValue[i] = *(myData + (uint32_t) localPeaksPosition[i]);
			TDOA_interPeak(PositionsArray, ValuesArray, localPeaksPosition[i],
					localMaxValue[i], step, &localPeaksPosition[i],
					&localMaxValue[i]);
		}
		TDOA_interPeak(localPeaksPosition, localMaxValue, localPeaksPosition[1],
				localMaxValue[1], step, peakEnvelope, maxEnvelope);
	}
	else
		return 0;       //signal error

	// return (0.379 * (*peakEnvelope + START_SAMPLES_POSTITION) - 8.734676667);
	return 1;
}

void TDOA_find3LocalPeaks(float32_t *myData, float32_t* LocalPeaksStoragePointer)
{
	uint32_t SamplePosition = 0;
	uint32_t maxSamplePosition = 0;
	int i;
	for (i = START_SAMPLES_POSTITION; i < NUM_DATAS; i++)
	{
		if (*(myData + i) > *(myData + maxSamplePosition))
		{
			maxSamplePosition = i;
		}
	}
	LocalPeaksStoragePointer[1] = maxSamplePosition;
	SamplePosition = TDOA_reachBottom(myData, LocalPeaksStoragePointer[1], -1);
	if (SamplePosition != 0)
	{
		LocalPeaksStoragePointer[0] = TDOA_reachPeak(myData, SamplePosition, -1);
	}
	SamplePosition = TDOA_reachBottom(myData, LocalPeaksStoragePointer[1], 1);
	if (SamplePosition != 0)
	{
		LocalPeaksStoragePointer[2] = TDOA_reachPeak(myData, SamplePosition, 1);
	}
}

uint32_t TDOA_reachBottom(float32_t *myData, uint32_t const PeakPosition,
		uint32_t const PointerIncreaseNumber)
{
	uint32_t SamplePosition = PeakPosition;
	while (SamplePosition > 1 && SamplePosition < NUM_DATAS)
	{
		if (*(myData + SamplePosition)
				< *(myData + SamplePosition + PointerIncreaseNumber))
		{
			return SamplePosition;
		}
		else
		{
			SamplePosition += PointerIncreaseNumber;
		}
	}
	SamplePosition = 0;
	return 0;
}

uint32_t TDOA_reachPeak(float32_t *myData, uint32_t const PeakPosition,
		uint32_t const PointerIncreaseNumber)
{
	uint32_t SamplePosition = PeakPosition;
	while (SamplePosition > 1 && SamplePosition < NUM_DATAS)
	{
		if (*(myData + SamplePosition)
				> *(myData + SamplePosition + PointerIncreaseNumber))
		{
			return SamplePosition;
		}
		else
		{
			SamplePosition += PointerIncreaseNumber;
		}
	}
	SamplePosition = 0;
	return 0;
}

void TDOA_interPeak(float32_t* PositionsArray, float32_t* ValuesArray,
		float32_t UserPosition, float32_t UserMaxValue, float32_t const step,
		float32_t* ReturnPosition, float32_t* ReturnValue)
{
	float32_t realLocalPeak = UserPosition;
	float32_t realLocalMax = UserMaxValue;
	float32_t samplePosition = realLocalPeak - step;
	float32_t interpolateValue = TDOA_larange(PositionsArray, ValuesArray,
			samplePosition);
	float32_t PointerDirection = 0;
	if (interpolateValue > UserMaxValue)
	{
		PointerDirection = -1;
		realLocalPeak = samplePosition;
		realLocalMax = interpolateValue;
	}
	else
	{
		PointerDirection = 1;
	}
	int flag = 1;
	while (flag)
	{
		samplePosition = realLocalPeak + step * PointerDirection;
		interpolateValue = TDOA_larange(PositionsArray, ValuesArray, samplePosition);
		if (interpolateValue >= realLocalMax)
		{
			realLocalMax = interpolateValue;
			realLocalPeak = samplePosition;
		}
		else
		{
			*ReturnPosition = realLocalPeak;
			*ReturnValue = realLocalMax;
			flag = 0;
		}
	}
}

float32_t TDOA_larange(float32_t *PositionsArray, float32_t *ValuesArray,
		float32_t interpolatePoint)
{
	float32_t result = 0;
	int i, j;
	float32_t temp;

	for (j = 0; j < 3; j++)
	{
		temp = 1;
		for (i = 0; i < 3; i++)
		{
			if (i != j)
				temp = (interpolatePoint - (*(PositionsArray + i))) * temp
						/ ((*(PositionsArray + j)) - (*(PositionsArray + i)));
		}
		result = result + (*(ValuesArray + j) * temp);
	}

	return result;
}
