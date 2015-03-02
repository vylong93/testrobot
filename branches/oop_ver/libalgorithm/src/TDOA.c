/*
 * TDOA.c
 *
 *  Created on: Sep 6, 2014
 *      Author: MasterE
 */

#include "libalgorithm/inc/TDOA.h"

static float FilterCoeffs[FILTER_ORDER] =
{ -0.003472f, 0.000573f, 0.006340f, 0.014220f, 0.022208f, 0.025940f, 0.020451f,
		0.002990f, -0.024527f, -0.054834f, -0.077140f, -0.081033f, -0.060950f,
		-0.019268f, 0.033526f, 0.081934f, 0.110796f, 0.110796f, 0.081934f,
		0.033526f, -0.019268f, -0.060950f, -0.081033f, -0.077140f, -0.054834f,
		-0.024527f, 0.002990f, 0.020451f, 0.025940f, 0.022208f, 0.014220f,
		0.006340f, 0.000573f, -0.003472f };

arm_fir_instance_f32 Filter;
float pState[STATE_BUFFER_SIZE] = { 0 };
float SamplesMic[NUMBER_OF_SAMPLE] = { 0 };
float OutputMic[NUMBER_OF_SAMPLE] = { 0 };

float g_f32Intercept = 1;
float g_f32Slope = 1;

// Pure Peak test =======================
void TDOA_process2(uint16_t* pui16ADCResult, float* pfPeakEnvelope, float* pfMaxEnvelope)
{
	// Filter signal: data output at OutputMic buffer
	TDOA_filterSignalsMic(pui16ADCResult);

	// Get pure peak
	*pfPeakEnvelope = TDOA_findPeak((float*)(OutputMic + START_SAMPLES_POSTITION));
	*pfMaxEnvelope = OutputMic[START_SAMPLES_POSTITION + (uint32_t)*pfPeakEnvelope];
}

float TDOA_findPeak(float *myData)
{
	uint32_t maxSamplePosition = 0;
	int i;
	for (i = 0; i < NUM_DATAS; i++)
	{
		if (*(myData + i) > *(myData + maxSamplePosition))
		{
			maxSamplePosition = i;
		}
	}
	return maxSamplePosition;
}
// ======================= Pure Peak test

void TDOA_initFilters()
{
	// Call FIR init function to initialize the instance structure.
	arm_fir_init_f32(&Filter, FILTER_ORDER, FilterCoeffs, pState, BLOCK_SIZE);
}

float TDOA_calculateDistanceFromTwoPeaks(float fPeakEnvelopeA, float fPeakEnvelopeB)
{
	float fDistanceA = (fPeakEnvelopeA - g_f32Intercept) / g_f32Slope;
	float fDistanceB = (fPeakEnvelopeB - g_f32Intercept) / g_f32Slope;

	float fSquareDistance = (((fDistanceA * fDistanceA + fDistanceB * fDistanceB) / 2.0)
					- (DISTANCE_BETWEEN_TWO_MICS_SQR / 4.0)) * 65536.0; // * 256^2

	return sqrtf(fSquareDistance);
}

void TDOA_process(uint16_t* pui16ADCResult, float* pfPeakEnvelope, float* pfMaxEnvelope)
{
	// Filter signal: data output at OutputMic buffer
	TDOA_filterSignalsMic(pui16ADCResult);

	// Run Alogrithm
	TDOA_getDistances((float*)(OutputMic + START_SAMPLES_POSTITION), pfPeakEnvelope, pfMaxEnvelope);
}

void TDOA_filterSignalsMic(uint16_t* pui16ADCResult)
{
	int32_t i;

	// Convert ui32ADCResult to SamplesMic
	for (i = 0; i < NUMBER_OF_SAMPLE; i++)
	{
		SamplesMic[i] = pui16ADCResult[i];
	}

	// Filter signals
	for (i = 0; i < NUM_BLOCKS; i++)
	{
		arm_fir_f32(&Filter, SamplesMic + (i * BLOCK_SIZE),
				OutputMic + (i * BLOCK_SIZE), BLOCK_SIZE);
	}
}

bool TDOA_isFilteredSignalNoisy(void)
{
	int i;
	for(i = START_SAMPLES_POSTITION; i < NUMBER_OF_SAMPLE; i++)
	{
		if((OutputMic[i] > NOISY_THRESHOLD) ||
			(OutputMic[i] < -NOISY_THRESHOLD))
			return true;
	}
	return false;
}

float TDOA_getDistances(float *myData, float *peakEnvelope, float *maxEnvelope)
{
	float step = +0.125f;
	float localPeaksPosition[3] =
	{ 0 };
	float localMaxValue[3] =
	{ 0 };

	TDOA_find3LocalPeaks(myData, localPeaksPosition);
	if (localPeaksPosition[0] != 0 && localPeaksPosition[1] != 0
			&& localPeaksPosition[2] != 0)
	{
		float PositionsArray[3] =
		{ 0 };
		float ValuesArray[3] =
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

void TDOA_find3LocalPeaks(float *myData, float* LocalPeaksStoragePointer)
{
	uint32_t SamplePosition = 0;
	uint32_t maxSamplePosition = 0;
	int i;
	for (i = 0; i < NUM_DATAS; i++)
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

uint32_t TDOA_reachBottom(float *myData, uint32_t const PeakPosition,
		int32_t const PointerIncreaseNumber)
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

uint32_t TDOA_reachPeak(float *myData, uint32_t const PeakPosition,
		int32_t const PointerIncreaseNumber)
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

void TDOA_interPeak(float* PositionsArray, float* ValuesArray,
		float UserPosition, float UserMaxValue, float const step,
		float* ReturnPosition, float* ReturnValue)
{
	float realLocalPeak = UserPosition;
	float realLocalMax = UserMaxValue;
	float samplePosition = realLocalPeak - step;
	float interpolateValue = TDOA_larange(PositionsArray, ValuesArray,
			samplePosition);
	float PointerDirection = 0;
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

float TDOA_larange(float *PositionsArray, float *ValuesArray,
		float interpolatePoint)
{
	float result = 0;
	int i, j;
	float temp;

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
