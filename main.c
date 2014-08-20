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
#include "inc/hw_i2c.h"
#include "inc/hw_eeprom.h"
#include "driverlib/debug.h"
#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/adc.h"
#include "driverlib/udma.h"
#include "driverlib/timer.h"
#include "driverlib/ssi.h"
#include "driverlib/i2c.h"
//#include "driverlib/rom.h"
#include "driverlib/pwm.h"
#include "driverlib/eeprom.h"

#include "libnrf24l01/inc/TM4C123_nRF24L01.h"
#include "libnrf24l01/inc/nRF24L01.h"
#include "CustomTivaDrivers.h"
#include "MainBoardDriver.h"

//*****************************************************************************
// System tick
//*****************************************************************************
#define SYSTICKS_PER_SECOND     1000
volatile uint32_t g_ui32SysTickCount;
void
SysTickHandler(void)
{
//  if (g_ui32SysTickCount == 1000)
//  { toggleLED(LED_RED);
//    g_ui32SysTickCount = 0;
//  }
//  else
  g_ui32SysTickCount++;
}

//*****************************************************************************
// The buffer to receive data from the RF module
//*****************************************************************************
uint8_t RF24_RX_buffer[32];

uint8_t TxAddrControlBoard[3];
uint8_t RxAddrControlBoard[3];

//*****************************************************************************
// Motors Speed
//*****************************************************************************
volatile uint8_t ui8LeftMotorDutyCycles;
volatile uint8_t ui8RightMotorDutyCycles;

//*****************************************************************************
// The buffers for ADCs
//*****************************************************************************
uint16_t g_ui16ADC0Result[NUMBER_OF_SAMPLE];
uint16_t g_ui16ADC1Result[NUMBER_OF_SAMPLE];
uint16_t g_ui16BatteryVoltage;

uint32_t g_ui32EEPROMAdderss;

int
main(void)
{
  // Internal OSC 16MHz source for PLL (400MHz/2)/4 = 50MHz sysclk
  //SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_INT);

  SysCtlClockSet(
  SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

//  // Set the system tick to fire 1000 times per second.
//  SysTickPeriodSet(SysCtlClockGet() / SYSTICKS_PER_SECOND);
//  SysTickIntEnable();
//  SysTickEnable();

  initEEPROM();

  initLED();

  initMotor();

  initRfModule();

  initDistanceSensingModules();

  initSpeaker();

  initBatteryChannel();

  initLowPowerMode();

  while (1)
  {
    rfDelayLoop(DELAY_CYCLES_5MS * 25);
    turnOnLED(LED_RED);
    rfDelayLoop(DELAY_CYCLES_5MS * 25);
    turnOffLED(LED_RED);
  }
}

inline void
RF24_IntHandler()
{
  if (GPIOIntStatus(RF24_INT_PORT, RF24_INT_Channel))
  {
    uint8_t rfIrqFlag = RF24_getIrqFlag(RF24_IRQ_MASK);

    if (rfIrqFlag & RF24_IRQ_TX)
    {
      RF24_clearIrqFlag(RF24_IRQ_TX);
      RF24_RX_activate();
    }

    if (rfIrqFlag & RF24_IRQ_MAX_RETRANS)
    {
      RF24_clearIrqFlag(RF24_IRQ_MAX_RETRANS);
      RF24_RX_activate();
    }

    if (rfIrqFlag & RF24_IRQ_RX)
    {
      uint8_t length;
      length = RF24_RX_getPayloadWidth();
      RF24_RX_getPayloadData(length, RF24_RX_buffer);

      // Commands allowed to be processed when uC is in sleep/deep_sleep mode
      if(CPUState != RUN_MODE)
      {
        switch (RF24_RX_buffer[0])
        {
          case COMMAND_SLEEP:
            CPUState = SLEEP_MODE;
            IntTrigger(INT_I2C1);
            break;

          case COMMAND_DEEP_SLEEP:
            CPUState = DEEP_SLEEP_MODE;
            IntTrigger(INT_I2C1);
            break;

          case COMMAND_WAKE_UP:
            wakeUpFormLPM();
            break;

          case COMMAND_RESET:
            HWREG(NVIC_APINT) = (NVIC_APINT_VECTKEY |
            NVIC_APINT_SYSRESETREQ);
            break;

          default:
            IntTrigger(INT_I2C1);
            break;
        }
      }
      else
      {
        switch (RF24_RX_buffer[0])
        {
          case PC_TEST_RF_TRANSMISSION:
            testRfTransmission();
            break;

          case PC_SEND_TEST_DATA_TO_PC:
            sendTestData();
            break;

          case PC_TEST_RF_CARRIER_DETECTION:
            testCarrierDetection();
            break;

          case PC_TOGGLE_ALL_STATUS_LEDS:
            GPIOPinToggle(LED_PORT_BASE, LED_ALL);
            break;

          case PC_TEST_ALL_MOTOR_MODES:
            testAllMotorModes();
            break;

          case PC_CHANGE_MOTORS_SPEED:
            disableMOTOR();
            PWMGenDisable(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_GEN);
            PWMGenDisable(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_GEN);

            setMotorDirection(LEFT_MOTOR_PORT_BASE, RF24_RX_buffer[1]);
            setMotorSpeed(LEFT_MOTOR_PWM_OUT1, RF24_RX_buffer[2]);
            setMotorDirection(RIGHT_MOTOR_PORT_BASE, RF24_RX_buffer[3]);
            setMotorSpeed(RIGHT_MOTOR_PWM_OUT1, RF24_RX_buffer[4]);

            PWMGenEnable(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_GEN);
            PWMGenEnable(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_GEN);

            PWMOutputState(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT1_BIT, true);
            PWMOutputState(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_OUT1_BIT, true);
            enableMOTOR();
            break;

          case PC_SEND_STOP_MOTOR_LEFT:
            PWMGenDisable(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_GEN);
            setMotorDirection(LEFT_MOTOR_PORT_BASE, FORWARD);
            PWMOutputState(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT1_BIT, false);
            break;

          case PC_SEND_STOP_MOTOR_RIGHT:
            PWMGenDisable(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_GEN);
            setMotorDirection(RIGHT_MOTOR_PORT_BASE, FORWARD);
            PWMOutputState(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_OUT1_BIT, false);
            break;

          case PC_SEND_DATA_ADC0_TO_PC:
            sendDataToControlBoard((uint8_t *) g_ui16ADC0Result);
            break;

          case PC_SEND_DATA_ADC1_TO_PC:
            sendDataToControlBoard((uint8_t *) g_ui16ADC1Result);
            break;

          case PC_SEND_BATT_VOLT_TO_PC:
            startSamplingBatteryVoltage();
            break;

          case PC_SEND_READ_EEPROM:
            readFormEEPROM();
            break;

          case PC_SEND_WRITE_EEPROM:
            writeToEEPROM();
            break;

          case PC_SEND_SET_ADDRESS_EEPROM:
            setAddressEEPROM();
            break;

          case PC_START_SAMPLING_MIC:
            startSamplingMicSignals();
            break;

          case PC_START_DISTANCE_SENSING:
            break;

          case PC_START_SPEAKER:
            startSpeaker();
            break;

          case COMMAND_SLEEP:
            CPUState = SLEEP_MODE;
            IntTrigger(INT_I2C1);
            break;

          case COMMAND_DEEP_SLEEP:
            CPUState = DEEP_SLEEP_MODE;
            IntTrigger(INT_I2C1);
            break;

          case COMMAND_WAKE_UP:
            wakeUpFormLPM();
            break;

          case COMMAND_RESET:
            HWREG(NVIC_APINT) = (NVIC_APINT_VECTKEY |
            NVIC_APINT_SYSRESETREQ);
            break;

          default:
            signalUnhandleError();
            break;
        }
      }
    }
    // Only clear the IRQ if the RF FIFO is empty
    if (RF24_RX_isEmpty())
    {
      RF24_clearIrqFlag(RF24_IRQ_RX);
      GPIOIntClear(RF24_INT_PORT, RF24_INT_Channel);
      GPIOIntClear(RF24_INT_PORT, RF24_INT_Channel);
    }
  }
}

void
I2C1_IntHandler(void)
{
  switch (CPUState)
  {
    case SLEEP_MODE:
      gotoSleepMode();
      break;
    case DEEP_SLEEP_MODE:
      gotoDeepSleepMode();
      break;
    default:
      break;
  }
}

