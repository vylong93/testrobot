/*
 * robot_analog.c
 *
 *  Created on: Jan 26, 2015
 *      Author: VyLong
 */

#include "librobot\inc\robot_analog.h"
#include "librobot\inc\robot_timer_delay.h"
#include "librobot\inc\robot_communication.h"
#include "libcustom\inc\custom_led.h"
#include "interrupt_definition.h"

//*****************************************************************************
// Random generator's variabless
//*****************************************************************************
static bool g_bIsGenerateRandomByteDone = false;
static uint8_t g_pui8RandomBuffer[8] =
{ 0 };
static uint8_t g_ui8RandomNumber = 0;
static uint32_t g_ui32uDMAErrCount = 0;

//*****************************************************************************
// Microphones signal's vairables
//*****************************************************************************
static bool g_bIsSamplingMic0Done = false;
static bool g_bIsSamplingMic1Done = false;
static uint16_t g_pui16ADC0Result[NUMBER_OF_SAMPLE] =
{ 0 };
static uint16_t g_pui16ADC1Result[NUMBER_OF_SAMPLE] =
{ 0 };

//*****************************************************************************
// Battery monitoring's vairables
//*****************************************************************************
static bool g_bSendToHost = false;
static uint16_t g_ui16BatteryVoltage = 0;

//*****************************************************************************
// uDMA's vairables
//*****************************************************************************
#ifdef gcc
static uint8_t ui8ControlTable[1024] __attribute__ ((aligned (1024)));
#else
#pragma DATA_ALIGN(ui8ControlTable, 1024)
static uint8_t ui8ControlTable[1024];
#endif

uint8_t* getMicrophone0BufferPointer(void)
{
	return (uint8_t*)g_pui16ADC0Result;
}
uint8_t* getMicrophone1BufferPointer(void)
{
	return (uint8_t*)g_pui16ADC1Result;
}

void initPeripheralsForAnalogFunction(void)
{
	/*
	 * Configure GPIO to analog type
	 *  ground adjacent pin to eliminate noise
	 */
	ROM_SysCtlPeripheralEnable(ADC_PORT_CLOCK);
	ROM_SysCtlDelay(2);
	ROM_GPIOPinTypeADC(ADC_PORT, ADC0_IN);
	ROM_GPIOPinTypeADC(ADC_PORT, ADC1_IN);

	ROM_GPIOPinTypeGPIOOutput(ADC_PORT, ADC_ADJACENT_PINS);
	ROM_GPIOPinWrite(ADC_PORT, ADC_ADJACENT_PINS, 0);

	/*
	 * Configure GPIO pin for battery monitoring
	 */
	ROM_SysCtlPeripheralEnable(BATTERY_PORT_CLOCK);
	ROM_SysCtlDelay(2);
	ROM_GPIOPinTypeADC(BATTERY_PORT, BATTERY_IN);

	/*
	 * Initialize ADC0
	 *  Sequence Type 2 use for sampling battery level
	 * 	Sequence Type 3 use for sampling microphone 1
	 */
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	ROM_ADCHardwareOversampleConfigure(ADC0_BASE, ADC_AVERAGING_FACTOR);

	ROM_ADCSequenceConfigure(ADC_BATT_BASE, ADC_BATT_SEQUENCE_TYPE,
	ADC_TRIGGER_PROCESSOR, BATTERY_MEASURENMENT_PRIORITY);
	ROM_ADCSequenceStepConfigure(ADC_BATT_BASE, ADC_BATT_SEQUENCE_TYPE, 0,
	BATTERY_CHANNEL | ADC_CTL_IE | ADC_CTL_END);
	ROM_ADCSequenceEnable(ADC_BATT_BASE, ADC_BATT_SEQUENCE_TYPE);

	ROM_ADCSequenceConfigure(ADC0_BASE, ADC_SEQUENCE_TYPE, ADC_TRIGGER_TIMER,
	DISTANCE_SENSING_PRIORITY);
	ROM_ADCSequenceStepConfigure(ADC0_BASE, ADC_SEQUENCE_TYPE, 0,
	ADC0_CHANNEL | ADC_CTL_IE | ADC_CTL_END);
	ROM_ADCSequenceEnable(ADC0_BASE, ADC_SEQUENCE_TYPE);
	ADCSequenceDMAEnable(ADC0_BASE, ADC_SEQUENCE_TYPE);

	/*
	 * Initialize ADC1
	 *  Sequence type 0 use for generate random number
	 *	Sequence Type 3 use for sampling microphone 2
	 */
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
	ROM_ADCHardwareOversampleConfigure(ADC1_BASE, ADC_AVERAGING_FACTOR);

	ROM_ADCSequenceConfigure(ADC_RANDOM_GEN_BASE, ADC_RANDOM_GEN_SEQUENCE_TYPE,
	ADC_TRIGGER_PROCESSOR, RANDOM_GEN_PRIORITY);
	ROM_ADCSequenceStepConfigure(ADC_RANDOM_GEN_BASE,
			ADC_RANDOM_GEN_SEQUENCE_TYPE, 0, RANDOM_GEN_CHANNEL);
	ROM_ADCSequenceStepConfigure(ADC_RANDOM_GEN_BASE,
			ADC_RANDOM_GEN_SEQUENCE_TYPE, 1, RANDOM_GEN_CHANNEL);
	ROM_ADCSequenceStepConfigure(ADC_RANDOM_GEN_BASE,
			ADC_RANDOM_GEN_SEQUENCE_TYPE, 2, RANDOM_GEN_CHANNEL);
	ROM_ADCSequenceStepConfigure(ADC_RANDOM_GEN_BASE,
			ADC_RANDOM_GEN_SEQUENCE_TYPE, 3, RANDOM_GEN_CHANNEL);
	ROM_ADCSequenceStepConfigure(ADC_RANDOM_GEN_BASE,
			ADC_RANDOM_GEN_SEQUENCE_TYPE, 4, RANDOM_GEN_CHANNEL);
	ROM_ADCSequenceStepConfigure(ADC_RANDOM_GEN_BASE,
			ADC_RANDOM_GEN_SEQUENCE_TYPE, 5, RANDOM_GEN_CHANNEL);
	ROM_ADCSequenceStepConfigure(ADC_RANDOM_GEN_BASE,
			ADC_RANDOM_GEN_SEQUENCE_TYPE, 6, RANDOM_GEN_CHANNEL);
	ROM_ADCSequenceStepConfigure(ADC_RANDOM_GEN_BASE,
			ADC_RANDOM_GEN_SEQUENCE_TYPE, 7,
			RANDOM_GEN_CHANNEL | ADC_CTL_IE | ADC_CTL_END);
	ROM_ADCSequenceEnable(ADC_RANDOM_GEN_BASE, ADC_RANDOM_GEN_SEQUENCE_TYPE);
	ADCSequenceDMAEnable(ADC_RANDOM_GEN_BASE, ADC_RANDOM_GEN_SEQUENCE_TYPE);

	ROM_ADCSequenceConfigure(ADC1_BASE, ADC_SEQUENCE_TYPE, ADC_TRIGGER_TIMER,
	DISTANCE_SENSING_PRIORITY);
	ROM_ADCSequenceStepConfigure(ADC1_BASE, ADC_SEQUENCE_TYPE, 0,
	ADC1_CHANNEL | ADC_CTL_IE | ADC_CTL_END);
	ROM_ADCSequenceEnable(ADC1_BASE, ADC_SEQUENCE_TYPE);
	ADCSequenceDMAEnable(ADC1_BASE, ADC_SEQUENCE_TYPE);

	/*
	 * Initialize uDMA
	 *  Channel 16 handle ADC0 Sequence type 2 - battery level
	 *	Channel 17 handle ADC0 Sequence type 3 - microphone 1
	 *	Channel 24 handle ADC1 Sequence type 0 - random number generator
	 *  Channel 27 handle ADC1 Sequence type 3 - microphone 2
	 */
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
	ROM_uDMAEnable();
	ROM_uDMAControlBaseSet(ui8ControlTable);

	ROM_uDMAChannelAssign(DMA_BATT_CHANNEL);
	ROM_uDMAChannelAttributeDisable(BATT_DMA_CHANNEL,
	UDMA_ATTR_ALTSELECT | UDMA_ATTR_HIGH_PRIORITY | UDMA_ATTR_REQMASK);
	ROM_uDMAChannelControlSet(BATT_DMA_CHANNEL | UDMA_PRI_SELECT,
	UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_NONE | UDMA_ARB_1);
	ROM_uDMAChannelTransferSet(BATT_DMA_CHANNEL | UDMA_PRI_SELECT,
			UDMA_MODE_BASIC,
			(void *) (ADC_BATT_BASE + ADC_BATT_SEQUENCE_ADDRESS),
			&g_ui16BatteryVoltage, 1);
	ROM_uDMAChannelEnable(BATT_DMA_CHANNEL);

	ROM_uDMAChannelAssign(DMA_ADC0_CHANNEL);
	ROM_uDMAChannelAttributeDisable(ADC0_DMA_CHANNEL,
	UDMA_ATTR_ALTSELECT | UDMA_ATTR_HIGH_PRIORITY | UDMA_ATTR_REQMASK);
	ROM_uDMAChannelControlSet(ADC0_DMA_CHANNEL | UDMA_PRI_SELECT,
	UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_1);
	ROM_uDMAChannelTransferSet(ADC0_DMA_CHANNEL | UDMA_PRI_SELECT,
			UDMA_MODE_BASIC, (void *) (ADC0_BASE + ADC_SEQUENCE_ADDRESS),
			g_pui16ADC0Result,
			NUMBER_OF_SAMPLE);
	ROM_uDMAChannelEnable(ADC0_DMA_CHANNEL);

	ROM_uDMAChannelAssign(DMA_RANDOM_GEN_CHANNEL);
	ROM_uDMAChannelAttributeDisable(RANDOM_GEN_DMA_CHANNEL,
	UDMA_ATTR_ALTSELECT | UDMA_ATTR_HIGH_PRIORITY | UDMA_ATTR_REQMASK);
	ROM_uDMAChannelControlSet(RANDOM_GEN_DMA_CHANNEL | UDMA_PRI_SELECT,
	UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_8 | UDMA_ARB_8);
	ROM_uDMAChannelTransferSet(RANDOM_GEN_DMA_CHANNEL | UDMA_PRI_SELECT,
	UDMA_MODE_BASIC,
			(void *) (ADC_RANDOM_GEN_BASE + RANDOM_GEN_SEQUENCE_ADDRESS),
			g_pui8RandomBuffer, 8);
	ROM_uDMAChannelEnable(RANDOM_GEN_DMA_CHANNEL);

	ROM_uDMAChannelAssign(DMA_ADC1_CHANNEL);
	ROM_uDMAChannelAttributeDisable(ADC1_DMA_CHANNEL,
	UDMA_ATTR_ALTSELECT | UDMA_ATTR_HIGH_PRIORITY | UDMA_ATTR_REQMASK);
	ROM_uDMAChannelControlSet(ADC1_DMA_CHANNEL | UDMA_PRI_SELECT,
	UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_1);
	ROM_uDMAChannelTransferSet(ADC1_DMA_CHANNEL | UDMA_PRI_SELECT,
			UDMA_MODE_BASIC, (void *) (ADC1_BASE + ADC_SEQUENCE_ADDRESS),
			g_pui16ADC1Result,
			NUMBER_OF_SAMPLE);
	ROM_uDMAChannelEnable(ADC1_DMA_CHANNEL);

	/*
	 * Configure Interrupts
	 */
	ROM_IntPrioritySet(ADC0_INT, PRIORITY_DMA_MIC1);
	ROM_IntPrioritySet(ADC1_INT, PRIORITY_DMA_MIC2);
	ROM_IntPrioritySet(ADC_BATT_INT, PRIORITY_DMA_BATT);
	ROM_IntPrioritySet(RANDOM_GEN_INT, PRIORITY_DMA_RANDOM_GEN);

	IntRegister(ADC0_INT, ADC0IntHandler);
	IntRegister(ADC1_INT, ADC1IntHandler);
	IntRegister(ADC_BATT_INT, BatterySequenceIntHandler);
	IntRegister(RANDOM_GEN_INT, RandomGeneratorIntHandler);
	IntRegister(INT_UDMAERR, uDMAErrorHandler);

	ROM_IntEnable(ADC0_INT);
	ROM_IntEnable(ADC1_INT);
	ROM_IntEnable(ADC_BATT_INT);
	ROM_IntEnable(RANDOM_GEN_INT);
	ROM_IntEnable(INT_UDMAERR);

	/*
	 * Configure Timer0 to trigger ADC0 Seq3 and ADC1 Seq3
	 */
	ROM_SysCtlPeripheralEnable(ADC_TIMER_CLOCK);
	ROM_TimerDisable(ADC_TIMER, TIMER_A);
	TimerClockSourceSet(ADC_TIMER, TIMER_CLOCK_SYSTEM);
	ROM_TimerConfigure(ADC_TIMER, TIMER_CFG_PERIODIC);
	ROM_TimerLoadSet(ADC_TIMER, TIMER_A,
			(ROM_SysCtlClockGet() / SAMPLE_FREQUENCY));
	ROM_TimerControlTrigger(ADC_TIMER, TIMER_A, true);
}

void triggerSamplingMicSignalsWithPreDelay(uint32_t ui32DelayUs)
{
	//TODO: uncomment
//	disableMOTOR();

	g_bIsSamplingMic0Done = false;
	g_bIsSamplingMic1Done = false;

	delay_timer_us(ui32DelayUs);	// DELAY_SAMPING_MICS_US

	ROM_TimerEnable(ADC_TIMER, TIMER_A);
}

void triggerSamplingBatteryVoltage(bool bIsSendToHost)
{
	g_bSendToHost = bIsSendToHost;

	//TODO: uncomment
//	disableMOTOR();

	ROM_ADCProcessorTrigger(ADC_BATT_BASE, ADC_BATT_SEQUENCE_TYPE);
}

float generateRandomFloatInRange(float min, float max)
{
	if (max <= min)
		return 0;

	float fResolution = 255.0f / (max - min);

	triggerGenerateRandomByte();
	while (!g_bIsGenerateRandomByteDone)
		;

	return ((float) (g_ui8RandomNumber / fResolution) + min);
}

void triggerGenerateRandomByte(void)
{
	g_bIsGenerateRandomByteDone = false;
	ROM_ADCProcessorTrigger(ADC_RANDOM_GEN_BASE, ADC_RANDOM_GEN_SEQUENCE_TYPE);
}

void ADC0IntHandler(void)
{
	uint32_t ui32Status;
	uint32_t ui32Mode;

	ui32Status = ROM_ADCIntStatus(ADC0_BASE, ADC_SEQUENCE_TYPE, true);
	ROM_ADCIntClear(ADC0_BASE, ui32Status);

	ui32Mode = ROM_uDMAChannelModeGet(ADC0_DMA_CHANNEL | UDMA_PRI_SELECT);
	if (ui32Mode == UDMA_MODE_STOP)
	{
		ROM_TimerDisable(ADC_TIMER, TIMER_A);
		// Setup for a future request
		ROM_uDMAChannelTransferSet(ADC0_DMA_CHANNEL | UDMA_PRI_SELECT,
		UDMA_MODE_BASIC, (void *) (ADC0_BASE + ADC_SEQUENCE_ADDRESS),
				g_pui16ADC0Result,
				NUMBER_OF_SAMPLE);
		ROM_uDMAChannelEnable(ADC0_DMA_CHANNEL);

		g_bIsSamplingMic0Done = true;
		if (g_bIsSamplingMic1Done)
		{
			//TODO: uncomment
//			enableMOTOR();
//			TDOA_run();
		}
	}
}

void ADC1IntHandler(void)
{
	uint32_t ui32Status;
	uint32_t ui32Mode;

	ui32Status = ROM_ADCIntStatus(ADC1_BASE, ADC_SEQUENCE_TYPE, true);
	ROM_ADCIntClear(ADC1_BASE, ui32Status);

	ui32Mode = ROM_uDMAChannelModeGet(ADC1_DMA_CHANNEL | UDMA_PRI_SELECT);
	if (ui32Mode == UDMA_MODE_STOP)
	{
		ROM_TimerDisable(ADC_TIMER, TIMER_A);
		// Setup for a future request
		ROM_uDMAChannelTransferSet(ADC1_DMA_CHANNEL | UDMA_PRI_SELECT,
		UDMA_MODE_BASIC, (void *) (ADC1_BASE + ADC_SEQUENCE_ADDRESS),
				g_pui16ADC1Result,
				NUMBER_OF_SAMPLE);
		ROM_uDMAChannelEnable(ADC1_DMA_CHANNEL);

		g_bIsSamplingMic1Done = true;

		if (g_bIsSamplingMic0Done)
		{
			//TODO: uncomment
//			enableMOTOR();
//			TDOA_run();
		}
	}
}

void BatterySequenceIntHandler(void)
{
	uint32_t ui32Status;
	uint32_t ui32Mode;

	ui32Status = ROM_ADCIntStatus(ADC_BATT_BASE, ADC_BATT_SEQUENCE_TYPE, true);
	ROM_ADCIntClear(ADC_BATT_BASE, ui32Status);

	ui32Mode = ROM_uDMAChannelModeGet(BATT_DMA_CHANNEL | UDMA_PRI_SELECT);
	if (ui32Mode == UDMA_MODE_STOP)
	{
		// Setup for a future request
		ROM_uDMAChannelTransferSet(BATT_DMA_CHANNEL | UDMA_PRI_SELECT,
		UDMA_MODE_BASIC, (void *) (ADC_BATT_BASE + ADC_BATT_SEQUENCE_ADDRESS),
				&g_ui16BatteryVoltage, 1);
		ROM_uDMAChannelEnable(BATT_DMA_CHANNEL);

		if (g_bSendToHost)
		{
			sendMessageToHost(MESSAGE_TYPE_ROBOT_RESPONSE, ROBOT_RESPONSE_OK,
					(uint8_t *) &g_ui16BatteryVoltage, 2);

			g_bSendToHost = false;

			turnOffLED(LED_GREEN);
		}

		//TODO: uncomment
//		enableMOTOR();
	}
}

void RandomGeneratorIntHandler(void)
{
	uint32_t ui32Status;
	uint32_t ui32Mode;

	ui32Status = ROM_ADCIntStatus(ADC_RANDOM_GEN_BASE,
			ADC_RANDOM_GEN_SEQUENCE_TYPE,
			true);
	ROM_ADCIntClear(ADC_RANDOM_GEN_BASE, ui32Status);

	ui32Mode = ROM_uDMAChannelModeGet(RANDOM_GEN_DMA_CHANNEL | UDMA_PRI_SELECT);
	if (ui32Mode == UDMA_MODE_STOP)
	{
		// Setup for a future request
		ROM_uDMAChannelTransferSet(RANDOM_GEN_DMA_CHANNEL | UDMA_PRI_SELECT,
		UDMA_MODE_BASIC,
				(void *) (ADC_RANDOM_GEN_BASE + RANDOM_GEN_SEQUENCE_ADDRESS),
				g_pui8RandomBuffer, 8);
		ROM_uDMAChannelEnable(RANDOM_GEN_DMA_CHANNEL);

		// Construct random byte from 8 samples
		g_ui8RandomNumber = 0;
		int i;
		for (i = 0; i < 8; i++)
		{
			g_ui8RandomNumber |= ((g_pui8RandomBuffer[i] & 0x01) << i);
		}

		// Perform random byte valid check
		if (g_ui8RandomNumber == 0 || g_ui8RandomNumber == 0xFF)
		{
			// Invalid randome byte, Trigger again
			ROM_ADCProcessorTrigger(ADC_RANDOM_GEN_BASE,
			ADC_RANDOM_GEN_SEQUENCE_TYPE);
		}
		else
		{
			g_bIsGenerateRandomByteDone = true;
		}
	}
}

void uDMAErrorHandler(void)
{
	uint32_t ui32Status;

	ui32Status = ROM_uDMAErrorStatusGet();

	if (ui32Status)
	{
		ROM_uDMAErrorStatusClear();
		g_ui32uDMAErrCount++;
	}
}
