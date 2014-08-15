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
#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/adc.h"
#include "driverlib/udma.h"
#include "driverlib/timer.h"
#include "driverlib/rom.h"
#include "driverlib/debug.h"

#include "CustomTivaDrivers.h"

void GPIOPinToggle(uint32_t ui32Port, uint8_t ui8Pins)
{
  ASSERT(_GPIOBaseValid(ui32Port));
  HWREG(ui32Port + (GPIO_O_DATA + (ui8Pins << 2))) ^= 0xFF;
}

void ADCDitherEnable(uint32_t ui32Base)
{
    ASSERT((ui32Base == ADC0_BASE) || (ui32Base == ADC1_BASE));

    HWREG(ui32Base + ADC_O_CTL) =
        (HWREG(ui32Base + ADC_O_CTL)) | ADC_CTL_DITHER;
}

void ADCDitherDisable(uint32_t ui32Base)
{
    ASSERT((ui32Base == ADC0_BASE) || (ui32Base == ADC1_BASE));

    HWREG(ui32Base + ADC_O_CTL) =
        (HWREG(ui32Base + ADC_O_CTL)) & (~ADC_CTL_DITHER);
}
