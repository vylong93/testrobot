/*
 * robot_analog.c
 *
 *  Created on: Jan 26, 2015
 *      Author: VyLong
 */


#include "librobot\inc\robot_analog.h"

unsigned char countAdcDMAsStopped = 0;

static uint32_t g_ui32uDMAErrCount = 0;

//*****************************************************************************
// The buffers for ADCs
//*****************************************************************************
uint16_t g_pui16ADC0Result[NUMBER_OF_SAMPLE];
uint16_t g_pui16ADC1Result[NUMBER_OF_SAMPLE];

uint16_t g_ui16BatteryVoltage;

#ifdef gcc
static uint8_t ui8ControlTable[1024] __attribute__ ((aligned (1024)));
#else
#pragma DATA_ALIGN(ui8ControlTable, 1024)
static uint8_t ui8ControlTable[1024];
#endif

inline void initPeripheralsForAnalogFunction(void)
{

	/*
	 * Configure GPIO to analog type
	 *  ground adjacent pin to eliminate noise
	 */
	SysCtlPeripheralEnable(ADC_PORT_CLOCK);
	SysCtlDelay(2);
	GPIOPinTypeADC(ADC_PORT, ADC0_IN);
	GPIOPinTypeADC(ADC_PORT, ADC1_IN);

	GPIOPinTypeGPIOOutput(ADC_PORT, ADC_ADJACENT_PINS);
	GPIOPinWrite(ADC_PORT, ADC_ADJACENT_PINS, 0);

	SysCtlPeripheralEnable(BATTERY_PORT_CLOCK);
	SysCtlDelay(2);
	GPIOPinTypeADC(BATTERY_PORT, BATTERY_IN);

	/*
	 * Initialize ADC0
	 *  Sequence Type 2 use for sampling battery level
	 * 	Sequence Type 3 use for sampling microphone 1
	 */
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	ADCHardwareOversampleConfigure(ADC0_BASE, ADC_AVERAGING_FACTOR);

	ADCSequenceConfigure(ADC_BATT_BASE, ADC_BATT_SEQUENCE_TYPE,
	ADC_TRIGGER_PROCESSOR, BATTERY_MEASURENMENT_PRIORITY);
	ADCSequenceStepConfigure(ADC_BATT_BASE, ADC_BATT_SEQUENCE_TYPE, 0,
	BATTERY_CHANNEL | ADC_CTL_IE | ADC_CTL_END);
	ADCSequenceEnable(ADC_BATT_BASE, ADC_BATT_SEQUENCE_TYPE);
	//ADCIntClear(ADC_BATT_BASE, ADC_BATT_SEQUENCE_TYPE);

	ADCSequenceConfigure(ADC0_BASE, ADC_SEQUENCE_TYPE, ADC_TRIGGER_TIMER,
	DISTANCE_SENSING_PRIORITY);
	ADCSequenceStepConfigure(ADC0_BASE, ADC_SEQUENCE_TYPE, 0,
	ADC0_CHANNEL | ADC_CTL_IE | ADC_CTL_END);
	ADCSequenceEnable(ADC0_BASE, ADC_SEQUENCE_TYPE);
	ADCSequenceDMAEnable(ADC0_BASE, ADC_SEQUENCE_TYPE);

	/*
	 * Initialize ADC1
	 *  Sequence type 0 use for generate random number
	 *	Sequence Type 3 use for sampling microphone 2
	 */
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
	ADCHardwareOversampleConfigure(ADC1_BASE, ADC_AVERAGING_FACTOR);

	ADCSequenceConfigure(ADC_RANDOM_GEN_BASE, ADC_RANDOM_GEN_SEQUENCE_TYPE,
	ADC_TRIGGER_PROCESSOR, RANDOM_GEN_PRIORITY);
	ADCSequenceStepConfigure(ADC_RANDOM_GEN_BASE, ADC_RANDOM_GEN_SEQUENCE_TYPE,
			0, RANDOM_GEN_CHANNEL);
	ADCSequenceStepConfigure(ADC_RANDOM_GEN_BASE, ADC_RANDOM_GEN_SEQUENCE_TYPE,
			1, RANDOM_GEN_CHANNEL);
	ADCSequenceStepConfigure(ADC_RANDOM_GEN_BASE, ADC_RANDOM_GEN_SEQUENCE_TYPE,
			2, RANDOM_GEN_CHANNEL);
	ADCSequenceStepConfigure(ADC_RANDOM_GEN_BASE, ADC_RANDOM_GEN_SEQUENCE_TYPE,
			3, RANDOM_GEN_CHANNEL);
	ADCSequenceStepConfigure(ADC_RANDOM_GEN_BASE, ADC_RANDOM_GEN_SEQUENCE_TYPE,
			4, RANDOM_GEN_CHANNEL);
	ADCSequenceStepConfigure(ADC_RANDOM_GEN_BASE, ADC_RANDOM_GEN_SEQUENCE_TYPE,
			5, RANDOM_GEN_CHANNEL);
	ADCSequenceStepConfigure(ADC_RANDOM_GEN_BASE, ADC_RANDOM_GEN_SEQUENCE_TYPE,
			6, RANDOM_GEN_CHANNEL);
	ADCSequenceStepConfigure(ADC_RANDOM_GEN_BASE, ADC_RANDOM_GEN_SEQUENCE_TYPE,
			7, RANDOM_GEN_CHANNEL | ADC_CTL_IE | ADC_CTL_END);
	ADCSequenceEnable(ADC_RANDOM_GEN_BASE, ADC_RANDOM_GEN_SEQUENCE_TYPE);
	ADCSequenceDMAEnable(ADC_RANDOM_GEN_BASE, ADC_RANDOM_GEN_SEQUENCE_TYPE);

	ADCSequenceConfigure(ADC1_BASE, ADC_SEQUENCE_TYPE, ADC_TRIGGER_TIMER,
	DISTANCE_SENSING_PRIORITY);
	ADCSequenceStepConfigure(ADC1_BASE, ADC_SEQUENCE_TYPE, 0,
	ADC1_CHANNEL | ADC_CTL_IE | ADC_CTL_END);
	ADCSequenceEnable(ADC1_BASE, ADC_SEQUENCE_TYPE);
	ADCSequenceDMAEnable(ADC1_BASE, ADC_SEQUENCE_TYPE);

	/*
	 * Initialize uDMA
	 *  Channel 16 handle ADC0 Sequence type 2 - battery level
	 *	Channel 17 handle ADC0 Sequence type 3 - microphone 1
	 *	Channel 24 handle ADC1 Sequence type 0 - random number generator
	 *  Channel 27 handle ADC1 Sequence type 3 - microphone 2
	 */
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
	uDMAEnable();
	uDMAControlBaseSet(ui8ControlTable);

	uDMAChannelAssign(DMA_BATT_CHANNEL);
	uDMAChannelAttributeDisable(BATT_DMA_CHANNEL,
	UDMA_ATTR_ALTSELECT | UDMA_ATTR_HIGH_PRIORITY | UDMA_ATTR_REQMASK);
	uDMAChannelControlSet(BATT_DMA_CHANNEL | UDMA_PRI_SELECT,
	UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_NONE | UDMA_ARB_1);
	uDMAChannelTransferSet(BATT_DMA_CHANNEL | UDMA_PRI_SELECT, UDMA_MODE_BASIC,
			(void *) (ADC_BATT_BASE + ADC_BATT_SEQUENCE_ADDRESS),
			&g_ui16BatteryVoltage, 1);
	uDMAChannelEnable(BATT_DMA_CHANNEL);

	uDMAChannelAssign(DMA_ADC0_CHANNEL);
	uDMAChannelAttributeDisable(ADC0_DMA_CHANNEL,
	UDMA_ATTR_ALTSELECT | UDMA_ATTR_HIGH_PRIORITY | UDMA_ATTR_REQMASK);
	uDMAChannelControlSet(ADC0_DMA_CHANNEL | UDMA_PRI_SELECT,
	UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_1);
	uDMAChannelTransferSet(ADC0_DMA_CHANNEL | UDMA_PRI_SELECT, UDMA_MODE_BASIC,
			(void *) (ADC0_BASE + ADC_SEQUENCE_ADDRESS), g_pui16ADC0Result,
			NUMBER_OF_SAMPLE);
	uDMAChannelEnable(ADC0_DMA_CHANNEL);

	uDMAChannelAssign(DMA_RANDOM_GEN_CHANNEL);
	uDMAChannelAttributeDisable(RANDOM_GEN_DMA_CHANNEL,
	UDMA_ATTR_ALTSELECT | UDMA_ATTR_HIGH_PRIORITY | UDMA_ATTR_REQMASK);
	uDMAChannelControlSet(RANDOM_GEN_DMA_CHANNEL | UDMA_PRI_SELECT,
	UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_8 | UDMA_ARB_8);
	uDMAChannelTransferSet(RANDOM_GEN_DMA_CHANNEL | UDMA_PRI_SELECT,
	UDMA_MODE_BASIC,
			(void *) (ADC_RANDOM_GEN_BASE + RANDOM_GEN_SEQUENCE_ADDRESS),
			g_pui8RandomBuffer, 8);
	uDMAChannelEnable(RANDOM_GEN_DMA_CHANNEL);

	uDMAChannelAssign(DMA_ADC1_CHANNEL);
	uDMAChannelAttributeDisable(ADC1_DMA_CHANNEL,
	UDMA_ATTR_ALTSELECT | UDMA_ATTR_HIGH_PRIORITY | UDMA_ATTR_REQMASK);
	uDMAChannelControlSet(ADC1_DMA_CHANNEL | UDMA_PRI_SELECT,
	UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_1);
	uDMAChannelTransferSet(ADC1_DMA_CHANNEL | UDMA_PRI_SELECT, UDMA_MODE_BASIC,
			(void *) (ADC1_BASE + ADC_SEQUENCE_ADDRESS), g_pui16ADC1Result,
			NUMBER_OF_SAMPLE);
	uDMAChannelEnable(ADC1_DMA_CHANNEL);

	/*
	 * Configure Interrupts
	 */
	IntPrioritySet(ADC0_INT, PRIORITY_DMA_MIC1);
	IntPrioritySet(ADC1_INT, PRIORITY_DMA_MIC2);
	IntPrioritySet(ADC_BATT_INT, PRIORITY_DMA_BATT);
	IntPrioritySet(RANDOM_GEN_INT, PRIORITY_DMA_RANDOM_GEN);

	IntEnable(ADC0_INT);
	IntEnable(ADC1_INT);
	IntEnable(ADC_BATT_INT);
	IntEnable(RANDOM_GEN_INT);

	IntEnable(INT_UDMAERR);

	/*
	 * Configure Timer to trigger ADC0 Seq3 and ADC1 Seq3
	 */
	SysCtlPeripheralEnable(ADC_TIMER_CLOCK);
	TimerDisable(ADC_TIMER, TIMER_A);
	TimerClockSourceSet(ADC_TIMER, TIMER_CLOCK_SYSTEM);
	TimerConfigure(ADC_TIMER, TIMER_CFG_PERIODIC);
	TimerLoadSet(ADC_TIMER, TIMER_A, (SysCtlClockGet() / SAMPLE_FREQUENCY));
	TimerControlTrigger(ADC_TIMER, TIMER_A, true);
}

inline void startSamplingMicSignals()
{
	disableMOTOR();
	countAdcDMAsStopped = 0;
	ROM_SysCtlDelay(DELAY_SAMPING_MIC);
	TimerEnable(ADC_TIMER, TIMER_A);
}
inline void startSamplingBatteryVoltage()
{
	disableMOTOR();
	ADCProcessorTrigger(ADC_BATT_BASE, ADC_BATT_SEQUENCE_TYPE);
}
inline void generateRandomByte()
{
	g_ui8RandomNumber = 0;
	ADCProcessorTrigger(ADC_RANDOM_GEN_BASE, ADC_RANDOM_GEN_SEQUENCE_TYPE);
}

float generateRandomRange(float min, float max)
{
	if (max <= min)
		return 0;

	float fResolution = 255.0f / (max - min);

	generateRandomByte();
	while (g_ui8RandomNumber == 0);

	return ((float)(g_ui8RandomNumber / fResolution) + min);
}

void ADC0IntHandler(void)
{
	uint32_t ui32Status;
	uint32_t ui32Mode;

	ui32Status = ADCIntStatus(ADC0_BASE, ADC_SEQUENCE_TYPE, true);
	ADCIntClear(ADC0_BASE, ui32Status);

	ui32Mode = uDMAChannelModeGet(ADC0_DMA_CHANNEL | UDMA_PRI_SELECT);
	if (ui32Mode == UDMA_MODE_STOP)
	{
		TimerDisable(ADC_TIMER, TIMER_A);
		// Setup for a future request
		uDMAChannelTransferSet(ADC0_DMA_CHANNEL | UDMA_PRI_SELECT,
		UDMA_MODE_BASIC, (void *) (ADC0_BASE + ADC_SEQUENCE_ADDRESS),
				g_pui16ADC0Result,
				NUMBER_OF_SAMPLE);
		uDMAChannelEnable(ADC0_DMA_CHANNEL);

		countAdcDMAsStopped++;
		if (countAdcDMAsStopped == 2)
		{
			enableMOTOR();
			TDOA_run();
		}
	}
}
void ADC1IntHandler(void)
{
	uint32_t ui32Status;
	uint32_t ui32Mode;

	ui32Status = ADCIntStatus(ADC1_BASE, ADC_SEQUENCE_TYPE, true);
	ADCIntClear(ADC1_BASE, ui32Status);

	ui32Mode = uDMAChannelModeGet(ADC1_DMA_CHANNEL | UDMA_PRI_SELECT);
	if (ui32Mode == UDMA_MODE_STOP)
	{
		TimerDisable(ADC_TIMER, TIMER_A);
		// Setup for a future request
		uDMAChannelTransferSet(ADC1_DMA_CHANNEL | UDMA_PRI_SELECT,
		UDMA_MODE_BASIC, (void *) (ADC1_BASE + ADC_SEQUENCE_ADDRESS),
				g_pui16ADC1Result,
				NUMBER_OF_SAMPLE);
		uDMAChannelEnable(ADC1_DMA_CHANNEL);

		countAdcDMAsStopped++;
		if (countAdcDMAsStopped == 2)
		{
			enableMOTOR();
			TDOA_run();
		}
	}
}
void BatterySequenceIntHandler(void)
{
	uint32_t ui32Status;
	uint32_t ui32Mode;

	ui32Status = ADCIntStatus(ADC_BATT_BASE, ADC_BATT_SEQUENCE_TYPE, true);
	ADCIntClear(ADC_BATT_BASE, ui32Status);

	ui32Mode = uDMAChannelModeGet(BATT_DMA_CHANNEL | UDMA_PRI_SELECT);
	if (ui32Mode == UDMA_MODE_STOP)
	{
		// Setup for a future request
		uDMAChannelTransferSet(BATT_DMA_CHANNEL | UDMA_PRI_SELECT,
		UDMA_MODE_BASIC, (void *) (ADC_BATT_BASE + ADC_BATT_SEQUENCE_ADDRESS),
				&g_ui16BatteryVoltage, 1);
		uDMAChannelEnable(BATT_DMA_CHANNEL);
		sendDataToControlBoard((uint8_t *) &g_ui16BatteryVoltage);
		enableMOTOR();
	}
}
void RandomGeneratorIntHandler(void)
{
	uint32_t ui32Status;
	uint32_t ui32Mode;

	ui32Status = ADCIntStatus(ADC_RANDOM_GEN_BASE, ADC_RANDOM_GEN_SEQUENCE_TYPE,
	true);
	ADCIntClear(ADC_RANDOM_GEN_BASE, ui32Status);

	ui32Mode = uDMAChannelModeGet(RANDOM_GEN_DMA_CHANNEL | UDMA_PRI_SELECT);
	if (ui32Mode == UDMA_MODE_STOP)
	{
		// Setup for a future request
		uDMAChannelTransferSet(RANDOM_GEN_DMA_CHANNEL | UDMA_PRI_SELECT,
		UDMA_MODE_BASIC,
				(void *) (ADC_RANDOM_GEN_BASE + RANDOM_GEN_SEQUENCE_ADDRESS),
				g_pui8RandomBuffer, 8);
		uDMAChannelEnable(RANDOM_GEN_DMA_CHANNEL);

		int i;
		for (i = 0; i < 8; i++)
		{
			g_ui8RandomNumber |= ((g_pui8RandomBuffer[i] & 0x01) << i);
		}

		if (g_ui8RandomNumber == 0 || g_ui8RandomNumber == 0xFF)
		{
			g_ui8RandomNumber = 0;
			ADCProcessorTrigger(ADC_RANDOM_GEN_BASE,
			ADC_RANDOM_GEN_SEQUENCE_TYPE);
		}
	}
}
void uDMAErrorHandler(void)
{
	uint32_t ui32Status;

	ui32Status = uDMAErrorStatusGet();

	if (ui32Status)
	{
		uDMAErrorStatusClear();
		g_ui32uDMAErrCount++;
	}
}
