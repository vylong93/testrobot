//  ***  modified by VyLong  *** //

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

inline void initSpiForRF(bool interruptEnable)
{
  // Enable the ports used by the RF board
  SysCtlPeripheralEnable(RF24_SPI_PORT_CLOCK);
  rfDelayLoop(1);
  if(RF24_SPI_PORT_CLOCK != RF24_INT_PORT_CLOCK)
  {
    SysCtlPeripheralEnable(RF24_INT_PORT_CLOCK);
    rfDelayLoop(1);
  }

  // Enable the SSI module used by the RF board
  SysCtlPeripheralEnable(RF24_SPI_CLOCK);
  rfDelayLoop(3);

  // Disable the SSI to config
  SSIDisable(RF24_SPI);
  rfDelayLoop(2);

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

  if(interruptEnable)
  {
          // Enable interrupt source of the interrupt pin used by RF board
          GPIOIntEnable(RF24_INT_PORT, RF24_INT_Channel);

          // Set the type of interrupt
          GPIOIntTypeSet(RF24_INT_PORT, RF24_INT_Pin, GPIO_FALLING_EDGE );

          // Enable interrupts to the processor.
          IntMasterEnable();

          // Enable the interrupts.
          IntEnable(RF24_INT);

          // Set the interrupt priorities.
          IntPrioritySet(RF24_INT, 0x00);
  }

  // Enable the SSI module.
  SSIEnable(RF24_SPI);

  uint32_t dataRx;
  // Read any residual data from the SSI port.
  while(SSIDataGetNonBlocking(RF24_SPI, &dataRx))
  {
  }
}

inline void  setRfCSN()
{
  GPIOPinWrite(RF24_SPI_PORT, RF24_CSN, RF24_CSN);
}

inline void  clearRfCSN()
{
  GPIOPinWrite(RF24_SPI_PORT, RF24_CSN, 0);
}

inline void  setRfCE()
{
  GPIOPinWrite(RF24_INT_PORT, RF24_CE, RF24_CE);
  CEState = 1;
}

inline void  clearRfCE()
{
  GPIOPinWrite(RF24_INT_PORT, RF24_CE, 0);
  CEState = 0;
}

char SPI_sendAndGetData(uint32_t inData)
{
    SSIDataPut(RF24_SPI, inData);
    uint32_t outData;
    SSIDataGet(RF24_SPI, &outData);
    return (char)outData;
}

void rfDelayLoop (uint32_t delay)
{
  SysCtlDelay(delay);
}

inline char getCEState()
{
  return CEState;
}

void disableRF24Interrupt() {
    // Disable the interrupts.
    IntDisable(RF24_INT);
}

void enableRF24Interrupt() {
    // Enable the interrupts.
    IntEnable(RF24_INT);
}
