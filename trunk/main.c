#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_nvic.h"
#include "inc/hw_gpio.h"
#include "driverlib/debug.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"

#define LED_CLOCK_PORT          SYSCTL_PERIPH_GPIOF
#define LED_PORT_BASE           GPIO_PORTF_BASE
#define LED_RED                 GPIO_PIN_1
#define LED_BLUE                GPIO_PIN_2
#define LED_GREEN               GPIO_PIN_3
#define LED_ALL                 (LED_RED | LED_GREEN | LED_BLUE)

int
main(void)
{
  // Internal OSC 16MHz source for PLL (400MHz/2)/4 = 50MHz sysclk
  ROM_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_INT);

  ROM_SysCtlPeripheralEnable(LED_CLOCK_PORT);
  ROM_GPIOPinTypeGPIOOutput(LED_PORT_BASE, LED_ALL);

  HWREG(LED_PORT_BASE + (GPIO_O_DATA + (LED_ALL << 2))) = 0x00;

  while (1)
  {
    ROM_SysCtlDelay(6250000);
    ASSERT(_GPIOBaseValid(LED_PORT_BASE));
    HWREG(LED_PORT_BASE + (GPIO_O_DATA + (LED_RED << 2))) ^= 0xFF;
  }
}
