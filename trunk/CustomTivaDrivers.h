#ifndef CUSTOM_TIVA_DRIVERS_H
#define CUSTOM_TIVA_DRIVERS_H

extern void GPIOPinToggle(uint32_t ui32Port, uint8_t ui8Pins);

extern void ADCDitherEnable(uint32_t ui32Base);

extern void ADCDitherDisable(uint32_t ui32Base);

#endif /* CUSTOM_TIVA_DRIVERS_H */
