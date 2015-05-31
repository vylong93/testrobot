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

float g_f32Intercept = DEFAULT_INTERCEPT_VALUE;
float g_f32Slope = DEFAULT_SLOPE_VALUE;

void TDOA_setIntercept(float fIntercept)
{
	g_f32Intercept = fIntercept;
}
float TDOA_getIntercept(void)
{
	return g_f32Intercept;
}

void TDOA_setSlope(float fSlope)
{
	g_f32Slope = fSlope;
}
float TDOA_getSlope(void)
{
	return g_f32Slope;
}

void TDOA_initFilters()
{
	// Call FIR init function to initialize the instance structure.
	arm_fir_init_f32(&Filter, FILTER_ORDER, FilterCoeffs, pState, BLOCK_SIZE);
}

bool TDOA_isGoodQualityMeasurement(float fMaxA, float fMaxB)
{
	return ((fMaxA > GOOD_QUALITY_THRESHOLD) && (fMaxB > GOOD_QUALITY_THRESHOLD));
}

uint16_t TDOA_calculateDistanceFromTwoPeaks(float fPeakEnvelopeA, float fPeakEnvelopeB, float fIntercept, float fSlope)
{
	//NOTE: output is Fixed-Point <8.8>

	float fDistanceA = ((fPeakEnvelopeA - fIntercept) / fSlope) + OFFSET_DISTANCE_IN_CM;
	float fDistanceB = ((fPeakEnvelopeB - fIntercept) / fSlope) + OFFSET_DISTANCE_IN_CM;

	float fSquareDistance = (((fDistanceA * fDistanceA
								+ fDistanceB * fDistanceB) / 2.0)
								- (DISTANCE_BETWEEN_TWO_MICS_SQR / 4.0))
								* 65536.0; // * 256^2

	fSquareDistance = sqrtf(fSquareDistance);

	if(fSquareDistance < MAXIMUM_DISTANCE)
		return (uint16_t)(fSquareDistance + 0.5);
	else
		return 0;
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

            if (PositionsArray[0] == 0 || PositionsArray[2] == (NUM_DATAS - 1))
                continue;

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
	float fMaxValue = myData[0];
	int i;
	for (i = 1; i < NUM_DATAS; i++)
	{
		if (myData[i] > fMaxValue)
		{
			fMaxValue = myData[i];
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
	while (SamplePosition >= 1 && SamplePosition < (NUM_DATAS - 1))
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

	return (SamplePosition - PointerIncreaseNumber);
}

uint32_t TDOA_reachPeak(float *myData, uint32_t const PeakPosition,
		int32_t const PointerIncreaseNumber)
{
	uint32_t SamplePosition = PeakPosition;
	while (SamplePosition >= 1 && SamplePosition < (NUM_DATAS - 1))
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

	return (SamplePosition - PointerIncreaseNumber);
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
