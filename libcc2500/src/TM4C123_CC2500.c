/*
 *  TM4C123_CC2500.c
 *
 *  Created on: Dec 31, 2014
 *      Author: VyLong
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>
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

#include "libcc2500/inc/cc2500.h"
#include "libcc2500/inc/TM4C123_CC2500.h"

static bool g_bIsIntEnable = false;

//----------------------------------------------------------------------------
//  void TI_CC_IRQ_handler()
//
//  DESCRIPTION:
//	If a packet is received, this function will be call to handle Rx data
//
//  TODO:
//	Define this function in your source code such as main.c
//----------------------------------------------------------------------------
extern void TI_CC_IRQ_handler();

//----------------------------------------------------------------------------
//  void TI_CC_SetCSN(void)
//
//  DESCRIPTION:
//  Disable chip select
//----------------------------------------------------------------------------
inline void TI_CC_SetCSN()
{
	ROM_GPIOPinWrite(CC2500_SPI_PORT, CC2500_CSN, CC2500_CSN);
}

//----------------------------------------------------------------------------
//  void TI_CC_ClearCSN(void)
//
//  DESCRIPTION:
//  Enable chip select
//----------------------------------------------------------------------------
inline void TI_CC_ClearCSN()
{
	ROM_GPIOPinWrite(CC2500_SPI_PORT, CC2500_CSN, 0);
}

//----------------------------------------------------------------------------
//  void TI_CC_Setup()
//
//  DESCRIPTION:
//  Configures the assigned interface to function as a SPI port and
//  initializes it.
//----------------------------------------------------------------------------
void TI_CC_Setup()
{
  // Enable the ports used by the RF board
  ROM_SysCtlPeripheralEnable(CC2500_SPI_PORT_CLOCK);
  TI_CC_Wait(1);

  if(CC2500_SPI_PORT_CLOCK != CC2500_INT_PORT_CLOCK)
  {
	ROM_SysCtlPeripheralEnable(CC2500_INT_PORT_CLOCK);
	TI_CC_Wait(1);
  }

  // Enable the SSI module used by the RF board
  ROM_SysCtlPeripheralEnable(CC2500_SPI_CLOCK);
  TI_CC_Wait(3);

  // Disable the SSI to config
  ROM_SSIDisable(CC2500_SPI);
  TI_CC_Wait(2);

  // Connect mux pins to the targeted SSI module
  ROM_GPIOPinConfigure(CC2500_SCK_CONFIGURE);
  ROM_GPIOPinConfigure(CC2500_MISO_CONFIGURE);
  ROM_GPIOPinConfigure(CC2500_MOSI_CONFIGURE);

  // Cofigure SSI pins
  ROM_GPIOPinTypeSSI(CC2500_SPI_PORT, CC2500_SCK | CC2500_MISO | CC2500_MOSI);

  // Configure the SSI port for SPI master mode.
  ROM_SSIConfigSetExpClk(CC2500_SPI, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, CC2500_SPI_BAUDRATE, 8);

  // Configure the interrupt pin as input
  ROM_GPIOPinTypeGPIOInput(CC2500_INT_PORT, CC2500_INT_Pin);

  // Configure GDO2
  ROM_GPIOPinTypeGPIOInput(CC2500_INT_PORT, CC2500_CCA_Pin);

  // We use soft SPI
  ROM_GPIOPinTypeGPIOOutput(CC2500_SPI_PORT, CC2500_CSN);

  // Clear interrupt flag
  GPIOIntClear(CC2500_INT_PORT, CC2500_INT_Pin);

  // Enable the SSI module.
  ROM_SSIEnable(CC2500_SPI);

  uint32_t dataRx;
  // Read any residual data from the SSI port.
  while(ROM_SSIDataGetNonBlocking(CC2500_SPI, &dataRx))
  {
  }
}

//----------------------------------------------------------------------------
//  void TI_CC_ConfigIRQPin(bool enable)
//
//  DESCRIPTION:
//  Configures the assigned interface to function as a SPI port and
//  initializes it.
//
//  ARGUMENTS:
//      bool enable
//          true/false for enable/disable interrupt at GPO0
//
//----------------------------------------------------------------------------
void TI_CC_ConfigIRQPin(bool enable)
{
  // Clear interrupt flag
  GPIOIntClear(CC2500_INT_PORT, CC2500_INT_Pin);

  if(enable)
  {
	  // Set the type of interrupt
	  ROM_GPIOIntTypeSet(CC2500_INT_PORT, CC2500_INT_Pin, GPIO_FALLING_EDGE);

	  // Set the interrupt priorities.
	  ROM_IntPrioritySet(CC2500_INT, 0x00);

	  // Register IRQ function handler
	  IntRegister(CC2500_INT, TI_CC_IRQ_handler);

	  // Enable interrupt source of the interrupt pin used by RF board
	  GPIOIntEnable(CC2500_INT_PORT, CC2500_INT_Channel);

	  // Clear pending interrupt request
	  ROM_IntPendClear(CC2500_INT);

	  // Enable the interrupts.
	  TI_CC_EnableInterrupt();
  }
  else
  {
	  // Disable the interrupts.
	  TI_CC_DisableInterrupt();
  }
}

//----------------------------------------------------------------------------
//  void TI_CC_SendAndGetData(char)
//
//  DESCRIPTION:
//  Transmit and get data in one transaction
//
//  ARGUMENTS:
//      char inData
//          data for transmittion
//
//----------------------------------------------------------------------------
char TI_CC_SendAndGetData(char inData)
{
	ROM_SSIDataPut(CC2500_SPI, (uint32_t)inData);
    uint32_t outData;
    ROM_SSIDataGet(CC2500_SPI, &outData);
    return (char)outData;
}

//----------------------------------------------------------------------------
//  void TI_CC_Wait(void)
//
//  DESCRIPTION:
//  delay cycles
//
//  ARGUMENTS:
//      unsigned int
//          number of delay microsecond: 1 ~ 1.02us
//
//----------------------------------------------------------------------------
void TI_CC_Wait(unsigned int cycles)
{
	ROM_SysCtlDelay((ROM_SysCtlClockGet() / (1000000 * 3)) * cycles);
}

//----------------------------------------------------------------------------
//  void TI_CC_WaitForCCxxxxReady(void)
//
//  DESCRIPTION:
//  spin until SO go low
//----------------------------------------------------------------------------
void TI_CC_WaitForCCxxxxReady(void)
{
	while (ROM_GPIOPinRead(CC2500_SPI_PORT, CC2500_MISO));	// Wait for CCxxxx ready
}

//----------------------------------------------------------------------------
//  void TI_CC_WaitForIntGoHigh(void)
//
//  DESCRIPTION:
//  spin until gpo0 go high
//----------------------------------------------------------------------------
void TI_CC_WaitForIntGoHigh(void)
{
	while (!ROM_GPIOPinRead(CC2500_INT_PORT, CC2500_INT_Pin));	// Wait GDO0 to go hi -> sync TX'ed
}

//----------------------------------------------------------------------------
//  void TI_CC_WaitForIntGoLow(void)
//
//  DESCRIPTION:
//  spin until gpo0 go low
//----------------------------------------------------------------------------
void TI_CC_WaitForIntGoLow(void)
{
	while (ROM_GPIOPinRead(CC2500_INT_PORT, CC2500_INT_Pin));	// Wait GDO0 to clear -> end of pkt
}

//----------------------------------------------------------------------------
//  void TI_CC_EnableInterrupt(void)
//
//  DESCRIPTION:
//  enable GDO0 interrupt
//----------------------------------------------------------------------------
void TI_CC_EnableInterrupt(void)
{
	ROM_IntEnable(CC2500_INT);
	g_bIsIntEnable = true;
}

//----------------------------------------------------------------------------
//  void TI_CC_DisableInterrupt(void)
//
//  DESCRIPTION:
//  disable GDO0 interrupt
//----------------------------------------------------------------------------
void TI_CC_DisableInterrupt(void)
{
	ROM_IntDisable(CC2500_INT);
	g_bIsIntEnable = false;
}

//----------------------------------------------------------------------------
//  void TI_CC_GetInterruptState(void)
//
//  DESCRIPTION:
//  return current interrupt status
//----------------------------------------------------------------------------
bool TI_CC_GetInterruptState(void)
{
	return g_bIsIntEnable;
}

//----------------------------------------------------------------------------
//  bool TI_CC_IsInterruptPinAsserted(void)
//
//  DESCRIPTION:
//  return current interrupt pin status
//----------------------------------------------------------------------------
bool TI_CC_IsInterruptPinAsserted(void)
{
	return ((GPIOIntStatus(CC2500_INT_PORT, false) & CC2500_INT_Pin) == CC2500_INT_Pin);
}

//----------------------------------------------------------------------------
//  void TI_CC_ClearIntFlag(void)
//
//  DESCRIPTION:
//  clear GDO0 interrupt flag
//----------------------------------------------------------------------------
void TI_CC_ClearIntFlag(void)
{
	GPIOIntClear(CC2500_INT_PORT, CC2500_INT_Pin);
}

//----------------------------------------------------------------------------
//  void TI_CC_ClearPending(void)
//
//  DESCRIPTION:
//  clear pending interrupt flag
//----------------------------------------------------------------------------
void TI_CC_ClearPending(void)
{
	ROM_IntPendClear(CC2500_INT);
}

//----------------------------------------------------------------------------
//  bool TI_CC_IsCRCOK(void)
//
//  DESCRIPTION:
//	get CRC return flag from CCxxx0 at GDO2 pin
//
//  RETURN VALUE:
//      bool
//			true: CRC OK
//			false: CRC ERROR
//----------------------------------------------------------------------------
bool TI_CC_IsCRCOK(void)
{
	return (ROM_GPIOPinRead(CC2500_INT_PORT, CC2500_CCA_Pin) == CC2500_CCA_Pin);
}

//----------------------------------------------------------------------------
//  void TI_CC_Strobe(char strobe)
//
//  DESCRIPTION:
//  Special write function for writing to command strobe registers.  Writes
//  to the strobe at address "addr".
//----------------------------------------------------------------------------
char TI_CC_Strobe(char strobe)
{
	char status;

	TI_CC_ClearCSN();     				 	// /CS enable

	TI_CC_WaitForCCxxxxReady();

    status = TI_CC_SendAndGetData(strobe);	// Send strobe

    TI_CC_SetCSN();       					// /CS disable

    return status;
}
