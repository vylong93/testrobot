#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_nvic.h"
#include "inc/hw_gpio.h"
#include "inc/hw_adc.h"
#include "inc/hw_udma.h"
#include "inc/hw_timer.h"
#include "inc/hw_ssi.h"
#include "driverlib/debug.h"
#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/adc.h"
#include "driverlib/udma.h"
#include "driverlib/timer.h"
#include "driverlib/ssi.h"
#include "driverlib/rom.h"
#include "driverlib/pwm.h"

#include "libnrf24l01/inc/TM4C123_nRF24L01.h"
#include "libnrf24l01/inc/nRF24L01.h"
#include "CustomTivaDrivers.h"
#include "MainBoardDriver.h"

//-----------------------LED functions-------------------------
inline void initLED()
{
    SysCtlPeripheralEnable(LED_CLOCK_PORT);
    GPIOPinTypeGPIOOutput(LED_PORT_BASE, LED_RED | LED_GREEN | LED_BLUE);
    turnOffLED(LED_RED | LED_GREEN | LED_BLUE);
}

inline void turnOnLED(uint8_t LEDpin) {
    ASSERT(_GPIOBaseValid(LED_PORT_BASE));
    HWREG(LED_PORT_BASE + (GPIO_O_DATA + (LEDpin << 2))) = LEDpin;
}

inline void turnOffLED(uint8_t LEDpin) {
    ASSERT(_GPIOBaseValid(LED_PORT_BASE));
    HWREG(LED_PORT_BASE + (GPIO_O_DATA + (LEDpin << 2))) = 0x00;
}

void signalUnhandleError()
{
    while(1)
    {
        GPIOPinWrite(LED_PORT_BASE, LED_ALL, LED_RED);
        SysCtlDelay(2000000);

        GPIOPinWrite(LED_PORT_BASE, LED_ALL, LED_GREEN);
        SysCtlDelay(2000000);

        GPIOPinWrite(LED_PORT_BASE, LED_ALL, LED_BLUE);
        SysCtlDelay(2000000);
    }
}
//------------------------------------------------LED functions


//-----------------------Motor functions-----------------------
static uint32_t ui32PWMPeriod;

inline void initMotor()
{
    SysCtlPWMClockSet(PWM_CLOCK_SELECT);
    SysCtlDelay(2);
    SysCtlPeripheralEnable(MOTOR_PWM_CLOCK);

    uint32_t pwmClock = SysCtlClockGet() / PWM_CLOCK_PRESCALE;
    ui32PWMPeriod = (pwmClock / MOTOR_PWM_FREQUENCY);

    SysCtlPeripheralEnable(MOTOR_SLEEP_PIN_CLOCK);
    SysCtlDelay(2);
    GPIOPinTypeGPIOOutput(MOTOR_SLEEP_PIN_BASE, MOTOR_SLEEP_PIN);

    SysCtlPeripheralEnable(LEFT_MOTOR_PORT_CLOCK);
    SysCtlDelay(2);
    GPIOPinTypeGPIOOutput(LEFT_MOTOR_PORT_BASE, LEFT_MOTOR_IN2);
    GPIOPinConfigure(LEFT_MOTOR_PWM_CONFIG);
    GPIOPinTypePWM(LEFT_MOTOR_PORT_BASE, LEFT_MOTOR_IN1);

    SysCtlPeripheralEnable(RIGHT_MOTOR_PORT_CLOCK);
    SysCtlDelay(2);
    GPIOPinTypeGPIOOutput(RIGHT_MOTOR_PORT_BASE, RIGHT_MOTOR_IN2);
    GPIOPinConfigure(RIGHT_MOTOR_PWM_CONFIG);
    GPIOPinTypePWM(RIGHT_MOTOR_PORT_BASE, RIGHT_MOTOR_IN1);

    PWMGenConfigure(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_GEN, PWM_GEN_MODE_DOWN);
    PWMGenPeriodSet(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_GEN, ui32PWMPeriod);
    PWMPulseWidthSet(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT1, 0);
    PWMOutputState(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT1_BIT, true);

    PWMGenConfigure(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_GEN, PWM_GEN_MODE_DOWN);  // M0PWM3
    PWMGenPeriodSet(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_GEN, ui32PWMPeriod);
    PWMPulseWidthSet(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_OUT1, 0);
    PWMOutputState(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_OUT1_BIT, true);
}

inline void enableMOTOR(){
    ASSERT(_GPIOBaseValid(MOTOR_SLEEP_PIN_BASE));
    HWREG(MOTOR_SLEEP_PIN_BASE + (GPIO_O_DATA + (MOTOR_SLEEP_PIN << 2))) = 0x01;
}

inline void disableMOTOR() {
    ASSERT(_GPIOBaseValid(MOTOR_SLEEP_PIN_BASE));
    HWREG(MOTOR_SLEEP_PIN_BASE + (GPIO_O_DATA + (MOTOR_SLEEP_PIN << 2))) = 0x00;
}

inline void setMotorDirection(uint32_t motorPortBase, uint8_t direction) {
    if (direction == FORWARD)
            GPIOPinWrite(motorPortBase, LEFT_MOTOR_IN2, 0);
    else if (direction == REVERSE)
            GPIOPinWrite(motorPortBase, LEFT_MOTOR_IN2, LEFT_MOTOR_IN2);
}

inline void testAllMotorModes()
{
    uint8_t motorDutyCycles;

    enableMOTOR();
    turnOnLED(LED_RED);

    //=========================Test LEFT Motor=========================
    setMotorDirection(LEFT_MOTOR_PORT_BASE, FORWARD);
    PWMGenEnable(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_GEN);

    // speed up
    turnOnLED(LED_GREEN);
    motorDutyCycles = 0;
    while (motorDutyCycles < 100) {
        motorDutyCycles++;
        PWMPulseWidthSet(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT1, motorDutyCycles * ui32PWMPeriod / 100);
        SysCtlDelay(500000);
    }

    // slow down
    turnOffLED(LED_GREEN);
    motorDutyCycles = 100;
    while (motorDutyCycles > 0) {
        motorDutyCycles--;
        PWMPulseWidthSet(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT1, motorDutyCycles * ui32PWMPeriod / 100);
        SysCtlDelay(500000);
    }

    setMotorDirection(LEFT_MOTOR_PORT_BASE, REVERSE);

    // speed up
    turnOnLED(LED_BLUE);
    motorDutyCycles = 100;
    while(motorDutyCycles > 0) {
        motorDutyCycles--;
        PWMPulseWidthSet(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT1, motorDutyCycles * ui32PWMPeriod / 100);
        SysCtlDelay(500000);
    }

    // slow down
    turnOffLED(LED_BLUE);
    motorDutyCycles = 0;
    while(motorDutyCycles < 100) {
        motorDutyCycles++;
        PWMPulseWidthSet(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT1, motorDutyCycles * ui32PWMPeriod / 100);
        SysCtlDelay(500000);
    }
    //==================================================Test LEFT Motor

    //=========================Test RIGHT Motor=========================
    setMotorDirection(RIGHT_MOTOR_PORT_BASE, FORWARD);
    PWMGenEnable(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_GEN);

    // speed up
    turnOnLED(LED_GREEN);
    motorDutyCycles = 0;
    while(motorDutyCycles < 100) {
            motorDutyCycles++;
            PWMPulseWidthSet(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_OUT1, motorDutyCycles * ui32PWMPeriod / 100);
            SysCtlDelay(500000);
    }

    // slow down
    turnOffLED(LED_GREEN);
    motorDutyCycles = 100;
    while(motorDutyCycles > 0) {
            motorDutyCycles--;
            PWMPulseWidthSet(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_OUT1, motorDutyCycles * ui32PWMPeriod / 100);
            SysCtlDelay(500000);
    }

    setMotorDirection(RIGHT_MOTOR_PORT_BASE, REVERSE);

    // speed up
    turnOnLED(LED_BLUE);
    motorDutyCycles = 100;
    while(motorDutyCycles > 0) {
            motorDutyCycles--;
            PWMPulseWidthSet(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_OUT1, motorDutyCycles * ui32PWMPeriod / 100);
            SysCtlDelay(500000);
    }

    // slow down
    turnOffLED(LED_BLUE);
    motorDutyCycles = 0;
    while(motorDutyCycles < 100) {
            motorDutyCycles++;
            PWMPulseWidthSet(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_OUT1, motorDutyCycles * ui32PWMPeriod / 100);
            SysCtlDelay(500000);
    }
    //==================================================Test RIGHT Motor

    //=========================Test BOTH Motors=========================
    setMotorDirection(RIGHT_MOTOR_PORT_BASE, FORWARD);
    setMotorDirection(LEFT_MOTOR_PORT_BASE, FORWARD);
    PWMGenEnable(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_GEN);
    PWMGenEnable(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_GEN);

    // speed up
    turnOnLED(LED_GREEN);
    motorDutyCycles = 0;
    while(motorDutyCycles < 100) {
            motorDutyCycles++;
            PWMPulseWidthSet(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_OUT1, motorDutyCycles * ui32PWMPeriod / 100);
            PWMPulseWidthSet(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT1, motorDutyCycles * ui32PWMPeriod / 100);
            SysCtlDelay(500000);
    }

    // slow down
    turnOffLED(LED_GREEN);
    motorDutyCycles = 100;
    while(motorDutyCycles > 0) {
            motorDutyCycles--;
            PWMPulseWidthSet(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_OUT1, motorDutyCycles * ui32PWMPeriod / 100);
            PWMPulseWidthSet(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT1, motorDutyCycles * ui32PWMPeriod / 100);
            SysCtlDelay(500000);
    }

    setMotorDirection(RIGHT_MOTOR_PORT_BASE, REVERSE);
    setMotorDirection(LEFT_MOTOR_PORT_BASE, REVERSE);
    PWMGenEnable(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_GEN);
    PWMGenEnable(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_GEN);

    // speed up
    turnOnLED(LED_BLUE);
    motorDutyCycles = 100;
    while(motorDutyCycles > 0) {
            motorDutyCycles--;
            PWMPulseWidthSet(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_OUT1, motorDutyCycles * ui32PWMPeriod / 100);
            PWMPulseWidthSet(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT1, motorDutyCycles * ui32PWMPeriod / 100);
            SysCtlDelay(500000);
    }

    // slow down
    turnOffLED(LED_BLUE);
    motorDutyCycles = 0;
    while(motorDutyCycles < 100) {
            motorDutyCycles++;
            PWMPulseWidthSet(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_OUT1, motorDutyCycles * ui32PWMPeriod / 100);
            PWMPulseWidthSet(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT1, motorDutyCycles * ui32PWMPeriod / 100);
            SysCtlDelay(500000);
    }
    //==================================================Test BOTH Motors

    disableMOTOR();
    turnOffLED(LED_RED);
}

inline void setMotorSpeed(uint32_t motorPortOut, uint8_t speed)
{
  PWMPulseWidthSet(MOTOR_PWM_BASE, motorPortOut, (speed * ui32PWMPeriod) / 100);
}
//----------------------------------------------Motor functions


//----------------Distance Sensing functions-------------------
static unsigned char countAdcDMAsStopped = 0;

static uint32_t g_ui32uDMAErrCount = 0;

#ifdef gcc
static uint8_t ui8ControlTable[1024] __attribute__ ((aligned (1024)));
#else
#pragma DATA_ALIGN(ui8ControlTable, 1024)
static uint8_t ui8ControlTable[1024];
#endif

void initADC(uint32_t adcClock, uint32_t adcBase, uint32_t adcChannel)
{
  SysCtlPeripheralEnable(adcClock);

  // Hardware Faulterror happens if enable this command.
  // The reason is unknown but it only happens when
  // we step over this function in the debug mode (or let it run consecutively)
  // ADCSequenceDisable(adcBase, ADC_SEQUENCE_TYPE);

  ADCHardwareOversampleConfigure(adcBase, ADC_AVERAGING_FACTOR);
  ADCSequenceConfigure(adcBase, ADC_SEQUENCE_TYPE, ADC_TRIGGER_TIMER, DISTANCE_SENSING_PRIORITY);
  ADCSequenceStepConfigure(adcBase, ADC_SEQUENCE_TYPE, 0, adcChannel | ADC_CTL_IE | ADC_CTL_END);
  ADCDitherEnable(adcBase);

  ADCSequenceEnable(adcBase, ADC_SEQUENCE_TYPE);
  ADCSequenceDMAEnable(adcBase, ADC_SEQUENCE_TYPE);
}

inline void groundAdjacentADCPins()
{
  GPIOPinTypeGPIOOutput(ADC_PORT, ADC_ADJACENT_PINS);
  GPIOPinWrite(ADC_PORT, ADC_ADJACENT_PINS, 0);
}

inline void initDistanceSensingModules(void)
{
  //=====================Init ADCs==========================
  SysCtlPeripheralEnable(ADC_PORT_CLOCK);
  GPIOPinTypeADC(ADC_PORT, ADC0_IN);
  GPIOPinTypeADC(ADC_PORT, ADC1_IN);
  initADC(SYSCTL_PERIPH_ADC0, ADC0_BASE, ADC0_CHANNEL);
  initADC(SYSCTL_PERIPH_ADC1, ADC1_BASE, ADC1_CHANNEL);
  groundAdjacentADCPins();
  //===============================================Init ADCs

  //=====================uDMA configure=====================
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
  uDMAEnable();
  uDMAControlBaseSet(ui8ControlTable);

  uDMAChannelAssign(DMA_ADC0_CHANNEL);
  uDMAChannelAttributeDisable(ADC0_DMA_CHANNEL, UDMA_ATTR_ALTSELECT | UDMA_ATTR_HIGH_PRIORITY | UDMA_ATTR_REQMASK);
  uDMAChannelControlSet(ADC0_DMA_CHANNEL | UDMA_PRI_SELECT, UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_1);
  uDMAChannelTransferSet(ADC0_DMA_CHANNEL | UDMA_PRI_SELECT, UDMA_MODE_BASIC,
      (void *) (ADC0_BASE + ADC_SEQUENCE_ADDRESS), g_ui16ADC0Result, NUMBER_OF_SAMPLE);

  uDMAChannelAssign(DMA_ADC1_CHANNEL);
  uDMAChannelAttributeDisable(ADC1_DMA_CHANNEL, UDMA_ATTR_ALTSELECT | UDMA_ATTR_HIGH_PRIORITY | UDMA_ATTR_REQMASK);
  uDMAChannelControlSet(ADC1_DMA_CHANNEL | UDMA_PRI_SELECT, UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_1);
  uDMAChannelTransferSet(ADC1_DMA_CHANNEL | UDMA_PRI_SELECT, UDMA_MODE_BASIC,
      (void *) (ADC1_BASE + ADC_SEQUENCE_ADDRESS), g_ui16ADC1Result, NUMBER_OF_SAMPLE);

  uDMAChannelEnable(ADC0_DMA_CHANNEL);
  uDMAChannelEnable(ADC1_DMA_CHANNEL);
  //==========================================uDMA configure

  // Interrupts Configure
  IntEnable(ADC0_INT);
  IntEnable(ADC1_INT);
  IntEnable(INT_UDMAERR);

  // ADC timer trigger configure
  SysCtlPeripheralEnable(ADC_TIMER_CLOCK);
  TimerDisable(ADC_TIMER, TIMER_A);
  TimerConfigure(ADC_TIMER, TIMER_CFG_PERIODIC);
  TimerLoadSet(ADC_TIMER, TIMER_A, (SysCtlClockGet() / SAMPLE_FREQUENCY));
  TimerControlTrigger(ADC_TIMER, TIMER_A, true);
}

inline void startSamplingMicSignals()
{
  disableMOTOR();
  countAdcDMAsStopped = 0;
  TimerEnable(ADC_TIMER, TIMER_A);
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
    uDMAChannelTransferSet(ADC0_DMA_CHANNEL | UDMA_PRI_SELECT, UDMA_MODE_BASIC,
        (void *) (ADC0_BASE + ADC_SEQUENCE_ADDRESS), g_ui16ADC0Result, NUMBER_OF_SAMPLE);
    uDMAChannelEnable(ADC0_DMA_CHANNEL);

    countAdcDMAsStopped++;
    if(countAdcDMAsStopped == 2)
        enableMOTOR();
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
    uDMAChannelTransferSet(ADC1_DMA_CHANNEL | UDMA_PRI_SELECT, UDMA_MODE_BASIC,
        (void *) (ADC1_BASE + ADC_SEQUENCE_ADDRESS), g_ui16ADC1Result, NUMBER_OF_SAMPLE);
    uDMAChannelEnable(ADC1_DMA_CHANNEL);

    countAdcDMAsStopped++;
    if(countAdcDMAsStopped == 2)
        enableMOTOR();
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
//-----------------------------------Distance Sensing functions

//----------------Battery measurement functions-------------------
inline void initBatteryChannel() {
  SysCtlPeripheralEnable(BATTERY_PORT_CLOCK);
  SysCtlDelay(2);
  GPIOPinTypeADC(BATTERY_PORT, BATTERY_IN);

  //=====================ADC configure=====================
  ADCSequenceConfigure(ADC_BATT_BASE, ADC_BATT_SEQUENCE_TYPE, ADC_TRIGGER_PROCESSOR, BATTERY_MEASURENMENT_PRIORITY);
  ADCSequenceStepConfigure(ADC_BATT_BASE, ADC_BATT_SEQUENCE_TYPE, 0, BATTERY_CHANNEL | ADC_CTL_IE | ADC_CTL_END);
  ADCSequenceEnable(ADC_BATT_BASE, ADC_BATT_SEQUENCE_TYPE);
  ADCIntClear(ADC_BATT_BASE, ADC_BATT_SEQUENCE_TYPE);
  //==========================================ADC configure

  //=====================uDMA configure=====================
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
  uDMAEnable();
  uDMAControlBaseSet(ui8ControlTable);

  uDMAChannelAssign(DMA_BATT_CHANNEL);
  uDMAChannelAttributeDisable(BATT_DMA_CHANNEL, UDMA_ATTR_ALTSELECT | UDMA_ATTR_HIGH_PRIORITY | UDMA_ATTR_REQMASK);
  uDMAChannelControlSet(BATT_DMA_CHANNEL | UDMA_PRI_SELECT, UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_NONE | UDMA_ARB_1);
  uDMAChannelTransferSet(BATT_DMA_CHANNEL | UDMA_PRI_SELECT, UDMA_MODE_BASIC,
      (void *) (ADC_BATT_BASE + ADC_BATT_SEQUENCE_ADDRESS), &g_ui16BatteryVoltage, 1);

  uDMAChannelEnable(BATT_DMA_CHANNEL);
  //==========================================uDMA configure

  // Interrupts Configure
  IntEnable(ADC_BATT_INT);
  IntEnable(INT_UDMAERR);
}

inline void startSamplingBatteryVoltage() {
  //disableMOTOR();
  ADCProcessorTrigger(ADC_BATT_BASE, ADC_BATT_SEQUENCE_TYPE);
}

void BatterySequenceIntHandler(void) {
  uint32_t ui32Status;
  uint32_t ui32Mode;

  ui32Status = ADCIntStatus(ADC_BATT_BASE, ADC_BATT_SEQUENCE_TYPE, true);
  ADCIntClear(ADC_BATT_BASE, ui32Status);

  ui32Mode = uDMAChannelModeGet(BATT_DMA_CHANNEL | UDMA_PRI_SELECT);
  if (ui32Mode == UDMA_MODE_STOP)
  {
    // Setup for a future request
	uDMAChannelTransferSet(BATT_DMA_CHANNEL | UDMA_PRI_SELECT, UDMA_MODE_BASIC,
	      (void *) (ADC_BATT_BASE + ADC_BATT_SEQUENCE_ADDRESS), &g_ui16BatteryVoltage, 1);
    uDMAChannelEnable(BATT_DMA_CHANNEL);
    //enableMOTOR();
  }
}
//-----------------------------------Battery measurement functions

//----------------Speaker functions-------------------
inline void initSpeaker()
{
  SysCtlPWMClockSet(PWM_CLOCK_SELECT);
  SysCtlDelay(2);
  SysCtlPeripheralEnable(SPEAKER_PWM_CLOCK_BASE);

//  SysCtlPeripheralEnable(SPEAKER_PORT_CLOCK);
  SysCtlDelay(2);

  if((SPEAKER_PORT_BASE == GPIO_PORTF_BASE) && (SPEAKER_PIN == GPIO_PIN_0))
  {
    // unlock the GPIO commit control register to modify PF0 configuration because it may be configured to be a NMI input.
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;
  }

  uint32_t pwmClock = SysCtlClockGet() / PWM_CLOCK_PRESCALE;
  uint32_t pwmPeriod = (pwmClock / SPEAKER_PWM_FREQUENCY);

  GPIOPinConfigure(SPEAKER_PWM_CONFIG);
  GPIODirModeSet(SPEAKER_PORT_BASE, SPEAKER_PIN, GPIO_DIR_MODE_HW);
  GPIOPadConfigSet(SPEAKER_PORT_BASE, SPEAKER_PIN, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);

  PWMGenConfigure(SPEAKER_PWM_BASE, SPEAKER_PWM_GEN, PWM_GEN_MODE_DOWN);
  PWMGenPeriodSet(SPEAKER_PWM_BASE, SPEAKER_PWM_GEN, pwmPeriod);
  PWMPulseWidthSet(SPEAKER_PWM_BASE, SPEAKER_PWM_OUT, pwmPeriod/2);


  SysCtlPeripheralEnable(SPEAKER_TIMER_CLOCK);
  TimerDisable(SPEAKER_TIMER_BASE, TIMER_A);
  TimerConfigure(SPEAKER_TIMER_BASE, TIMER_CFG_ONE_SHOT);
  TimerLoadSet(SPEAKER_TIMER_BASE, TIMER_A, (SysCtlClockGet() / SPEAKER_TIMER_FREQUENCY));
  IntMasterEnable();
  TimerIntEnable(SPEAKER_TIMER_BASE, TIMER_TIMA_TIMEOUT);
  IntEnable(SPEAKER_INT);
}

void startSpeaker()
{
  PWMOutputState(SPEAKER_PWM_BASE, SPEAKER_PWM_OUT_BIT, true);
  PWMGenEnable(SPEAKER_PWM_BASE, SPEAKER_PWM_GEN);
  TimerEnable(SPEAKER_TIMER_BASE, TIMER_A);
}

void SpeakerTimerIntHandler(void)
{
  TimerIntClear(SPEAKER_TIMER_BASE, TIMER_TIMA_TIMEOUT);
  PWMGenDisable(SPEAKER_PWM_BASE, SPEAKER_PWM_GEN);
  PWMSyncTimeBase(SPEAKER_PWM_BASE, SPEAKER_PWM_GEN_BIT);
  PWMOutputState(SPEAKER_PWM_BASE, SPEAKER_PWM_OUT_BIT, false);
}
//-----------------------------------Speaker functions

//----------------------RF24 Functions------------------------
extern uint8_t RF24_RX_buffer[32];

inline void initRfModule()
{
  RF24_InitTypeDef initRf24;
  initRf24.AddressWidth = RF24_ADRESS_WIDTH_3;
  initRf24.Channel = RF24_CHANNEL_0;
  initRf24.CrcBytes = RF24_CRC_2BYTES;
  initRf24.CrcState = RF24_CRC_EN;
  initRf24.RetransmitCount = RF24_RETRANS_COUNT15;
  initRf24.RetransmitDelay = RF24_RETRANS_DELAY_1000u;
  initRf24.Speed = RF24_SPEED_1MBPS;
  initRf24.Power = RF24_POWER_0DBM;
  initRf24.Features = RF24_FEATURE_EN_DYNAMIC_PAYLOAD | RF24_FEATURE_EN_NO_ACK_COMMAND;
  initRf24.InterruptEnable = true;
  initRf24.LNAGainEnable = true;
  RF24_init(&initRf24);

  // Set payload pipe#0 dynamic
  RF24_PIPE_setPacketSize(RF24_PIPE0, RF24_PACKET_SIZE_DYNAMIC);

  // Open pipe#0 with Enhanced ShockBurst enabled for receiving Auto-ACKs
  RF24_PIPE_open(RF24_PIPE0, true);

  uint8_t addr[3] =  {0xDE, 0xAD, 0xBE};
  RF24_RX_setAddress(RF24_PIPE0, addr);
  RF24_TX_setAddress(addr);

  RF24_RX_activate();
}

void sendDataToControlBoard(uint8_t * data)
{
  GPIOPinWrite(LED_PORT_BASE, LED_ALL, LED_GREEN);

  uint32_t length;
  length = RF24_RX_buffer[1];
  length = length << 8;
  length = (length << 8) + RF24_RX_buffer[2];
  length = (length << 8) + RF24_RX_buffer[3];
  length = (length << 8) + RF24_RX_buffer[4];

  uint8_t buffer[32];
  uint32_t pointer = 0;
  uint32_t i;

  RF24_RX_flush();
  RF24_clearIrqFlag(RF24_IRQ_RX);
  RF24_TX_activate();
  RF24_RETRANS_setCount(RF24_RETRANS_COUNT15);
  RF24_RETRANS_setDelay(RF24_RETRANS_DELAY_250u);
  while(1)
  {
    for( i = 0; (i<length) && (i<32); i++)
    {
      buffer[i] = *(data + pointer);
      pointer++;
    }
    RF24_TX_writePayloadAck(i, &buffer[0]);
    RF24_TX_pulseTransmit();

    while(1)
    {
       if(GPIOPinRead(RF24_INT_PORT, RF24_INT_Pin) == 0)
       {
         if(RF24_getIrqFlag(RF24_IRQ_TX))
           break;
         if(RF24_getIrqFlag(RF24_IRQ_MAX_RETRANS))
         {
           RF24_clearIrqFlag(RF24_IRQ_MAX_RETRANS);
           RF24_RX_activate();
           return;
         }
       }
    }
    RF24_clearIrqFlag(RF24_IRQ_TX);

    if(length > 32)
      length -= 32;
    else
    {
        RF24_RX_activate();
        GPIOPinWrite(LED_PORT_BASE, LED_ALL, LED_RED);
      return;
    }

  }
}

inline void testCarrierDetection()
{
  rfDelayLoop(DELAY_CYCLES_5MS);
  while(RF24_RX_carrierDetection())
  {
    GPIOPinWrite(LED_PORT_BASE, LED_BLUE, LED_BLUE);
    rfDelayLoop(DELAY_CYCLES_5MS*25);
    GPIOPinWrite(LED_PORT_BASE, LED_BLUE, 0);
    rfDelayLoop(DELAY_CYCLES_5MS*25);
  }
}

inline void testRfTransmission()
{
  GPIOPinWrite(LED_PORT_BASE, LED_ALL, LED_GREEN);

  uint32_t length;
  uint8_t dataLength;
  length = RF24_RX_buffer[1];
  length = length << 8;
  length = (length << 8) | RF24_RX_buffer[2];
  length = (length << 8) | RF24_RX_buffer[3];
  length = (length << 8) | RF24_RX_buffer[4];

  uint8_t value = 0;
  uint32_t i;

  while(1)
  {
    RF24_clearIrqFlag(RF24_IRQ_RX);

    while(RF24_getIrqFlag(RF24_IRQ_RX) == 0)
      ;

    dataLength = RF24_RX_getPayloadWidth();
    RF24_RX_getPayloadData(dataLength, RF24_RX_buffer);

    for(i = 0; (i<length) && (i<32); i++)
    {
      if(RF24_RX_buffer[i] != value)
      {
          GPIOPinWrite(LED_PORT_BASE, LED_RED, LED_RED);
        return;
      }
      value++;
    }

    if(length > 32)
      length -= 32;
    else
    {
      GPIOPinWrite(LED_PORT_BASE, LED_ALL, LED_RED);
      return;
    }

  }
}

inline void sendTestData()
{
  uint32_t i;
  uint16_t testData[NUMBER_OF_SAMPLE];
  for(i = 0; i < NUMBER_OF_SAMPLE; i++)
  {
      testData[i] = i;
  }
  sendDataToControlBoard((uint8_t *) testData);
}
//----------------------------------------------RF24 Functions

//----------------Low Power Mode functions-------------------
int8_t CPUState = RUN_MODE;

inline void initLowPowerMode() {
    //
	// Wake up condition form Sleep/Deep Sleep mode:
	// Any interrupt events will force CPU back to Run Mode.
	//
	
	//==========================
	// SLEEP MODE Configuration
	//==========================
    // In Sleep mode, the clock frequency of the active peripherals is unchanged.
	
	//
    // Enable Peripherals in Sleep Mode.
    //
	SysCtlPeripheralSleepEnable(RF24_INT_PORT_CLOCK);	// IMPORTANCE: allow IRQ pin of RF module wake CPU up when new byte has received.
    SysCtlPeripheralSleepEnable(MOTOR_PWM_CLOCK);			// keep PWM operate in Sleep Mode for Motor control.
	SysCtlPeripheralSleepEnable(MOTOR_SLEEP_PIN_CLOCK); 	//TODO: use PWM out instead of GPIO
	SysCtlPeripheralSleepEnable(RIGHT_MOTOR_PORT_CLOCK);	//TODO: use PWM out instead of GPIO

	//
    // Set LDO to 1.15V in Sleep.
	// Available options: 0.9V, 0.95V, 1V, 1.05V, 1.1V, 1.15V, 1.2V
    //
    SysCtlLDOSleepSet(SYSCTL_LDO_1_15V);
	
	//
    // Set SRAM to Standby when in Sleep Mode.
    //
    SysCtlSleepPowerSet(SYSCTL_SRAM_STANDBY);
	
	//==============================
	// DEEP SLEEP MODE Configuration
	//==============================
	//
    // Set the clocking for Deep-Sleep.
    // Power down the PIOSC & MOSC to save power and run from the
    // internal 30kHz osc.
    //
    SysCtlDeepSleepClockConfigSet(1, (SYSCTL_DSLP_OSC_INT30 |
                SYSCTL_DSLP_PIOSC_PD | SYSCTL_DSLP_MOSC_PD));		
			
    //
    // Enable Peripherals in Deep-Sleep Mode.
    //
    SysCtlPeripheralDeepSleepEnable(RF24_INT_PORT_CLOCK);	// IMPORTANCE: allow IRQ pin of RF module wake CPU up when new byte has received.

	//
    // Set LDO to 0.9V in Deep-Sleep.
    // Available options: 0.9V, 0.95V, 1V, 1.05V, 1.1V, 1.15V, 1.2V
	// 
    SysCtlLDODeepSleepSet(SYSCTL_LDO_0_90V);
	
	//
    // Set Flash & SRAM to Low Power in Deep-Sleep Mode.
    //
    SysCtlDeepSleepPowerSet(SYSCTL_FLASH_LOW_POWER | SYSCTL_SRAM_LOW_POWER);

	//==============================================
    // IMPORTANCE: Enable Auto Clock Gating Control.
    //==============================================
    SysCtlPeripheralClockGating(true);

    CPUState = RUN_MODE;
}

inline void gotoSleepMode() {
	//
    // NOTE: Switch clock to PIOSC and power down the MOSC before going into Sleep.
	// This will be the Run mode's clock configuration after wake up form Sleep mode. 
	// So that reconfigure system clock should be considered if required.
	//
    SysCtlClockSet(SYSCTL_OSC_INT | SYSCTL_USE_OSC | SYSCTL_MAIN_OSC_DIS);

    turnOffLED(LED_ALL);
    turnOnLED(LED_BLUE);

	SysCtlSleep();
}

inline void gotoDeepSleepMode() {
    turnOffLED(LED_ALL);
    //turnOnLED(LED_GREEN);

	SysCtlDeepSleep();
}

inline void wakeUpFormLPM() {
    if (CPUState == SLEEP_MODE) {
		// Switch back to the IOSC + PLL (50MHz sysclk)
		SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_INT);
    }
    CPUState = RUN_MODE;
}

//----------------------------------Low Power Mode Functions
