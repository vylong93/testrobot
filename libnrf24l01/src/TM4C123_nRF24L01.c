//  ***  modified by VyLong  *** //

#ifdef RF_USE_nRF24L01

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_nvic.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/ssi.h"

#include "libnrf24l01/inc/nRF24L01.h"
#include "libnrf24l01/inc/TM4C123_nRF24L01.h"

// This variable hold the state of CE.
static char CEState;

static bool g_bIsIntEnable = false;

extern void MCU_RF_IRQ_handler();


void MCU_RF_InitSpiForRf(void)
{
  // Enable the ports used by the RF board
  SysCtlPeripheralEnable(RF24_SPI_PORT_CLOCK);
  MCU_RF_WaitUs(1);
  if(RF24_SPI_PORT_CLOCK != RF24_INT_PORT_CLOCK)
  {
    SysCtlPeripheralEnable(RF24_INT_PORT_CLOCK);
    MCU_RF_WaitUs(1);
  }

  // Enable the SSI module used by the RF board
  SysCtlPeripheralEnable(RF24_SPI_CLOCK);
  MCU_RF_WaitUs(3);

  // Disable the SSI to config
  SSIDisable(RF24_SPI);
  MCU_RF_WaitUs(2);

  // Connect mux pins to the targeted SSI module
  GPIOPinConfigure(RF24_SCK_CONFIGURE);
  GPIOPinConfigure(RF24_MISO_CONFIGURE);
  GPIOPinConfigure(RF24_MOSI_CONFIGURE);

  // Cofigure SSI pins
  GPIOPinTypeSSI(RF24_SPI_PORT, RF24_SCK | RF24_MISO | RF24_MOSI);

  // Configure the SSI port for SPI master mode.
  SSIConfigSetExpClk(RF24_SPI, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, RF24_SPI_BAUDRATE, 8);

  // Configure the interrupt pin as input
  GPIOPinTypeGPIOInput(RF24_INT_PORT, RF24_INT_Pin);

  //We use soft SPI
  GPIOPinTypeGPIOOutput(RF24_SPI_PORT, RF24_CSN);

  // Configure the CE pin
  GPIOPinTypeGPIOOutput(RF24_INT_PORT, RF24_CE);

  // Enable the SSI module.
  SSIEnable(RF24_SPI);

  uint32_t dataRx;
  // Read any residual data from the SSI port.
  while(SSIDataGetNonBlocking(RF24_SPI, &dataRx))
  {
  }
}

void MCU_RF_ConfigIRQPin(bool bEnable)
{
  // Clear interrupt flag
  GPIOIntClear(RF24_INT_PORT, RF24_INT_Pin);

  if(bEnable)
  {
	  // Set the type of interrupt
	  ROM_GPIOIntTypeSet(RF24_INT_PORT, RF24_INT_Pin, GPIO_FALLING_EDGE );

	  // Set the interrupt priorities.
	  ROM_IntPrioritySet(RF24_INT, 0x00);

	  // Register IRQ function handler
	  IntRegister(RF24_INT, MCU_RF_IRQ_handler);

	  // Enable interrupt source of the interrupt pin used by RF board
	  GPIOIntEnable(RF24_INT_PORT, RF24_INT_Channel);

	  // Clear pending interrupt request
	  ROM_IntPendClear(RF24_INT);

	  // Enable the interrupts.
	  MCU_RF_EnableInterrupt();
  }
}

void MCU_RF_WaitUs(unsigned int cycles)
{
	ROM_SysCtlDelay((ROM_SysCtlClockGet() / (1000000 * 3)) * cycles);
}


bool MCU_RF_GetInterruptState(void)
{
	return g_bIsIntEnable;
}

void MCU_RF_EnableInterrupt(void)
{
	ROM_IntEnable(RF24_INT);
	g_bIsIntEnable = true;
}

void MCU_RF_DisableInterrupt(void)
{
	ROM_IntDisable(RF24_INT);
	g_bIsIntEnable = false;
}

bool MCU_RF_IsInterruptPinAsserted(void)
{
	return ((GPIOIntStatus(RF24_INT_PORT, false) & RF24_INT_Pin) == RF24_INT_Pin);
}

void MCU_RF_ClearIntFlag(void)
{
	GPIOIntClear(RF24_INT_PORT, RF24_INT_Pin);
}

void MCU_RF_ClearPending(void)
{
	ROM_IntPendClear(RF24_INT);
}


void MCU_RF_SetCSN()
{
  ROM_GPIOPinWrite(RF24_SPI_PORT, RF24_CSN, RF24_CSN);
}

void MCU_RF_ClearCSN()
{
  ROM_GPIOPinWrite(RF24_SPI_PORT, RF24_CSN, 0);
}

void MCU_RF_SetCE()
{
  ROM_GPIOPinWrite(RF24_INT_PORT, RF24_CE, RF24_CE);
  CEState = 1;
}

void MCU_RF_ClearCE()
{
  ROM_GPIOPinWrite(RF24_INT_PORT, RF24_CE, 0);
  CEState = 0;
}

char MCU_RF_SendAndGetData(uint32_t inData)
{
    ROM_SSIDataPut(RF24_SPI, inData);
    uint32_t outData;
    ROM_SSIDataGet(RF24_SPI, &outData);
    return (char)outData;
}


char MCU_RF_GetCEState()
{
  return CEState;
}


#endif
