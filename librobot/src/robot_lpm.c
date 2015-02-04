/*
 * robot_lpm.c
 *
 *  Created on: Jan 26, 2015
 *      Author: VyLong
 */

#include "librobot/inc/robot_lpm.h"

#include "libcustom/inc/custom_led.h"
#include "libcustom/inc/custom_delay.h"
#include "libcustom/inc/custom_uart_debug.h"

#ifdef RF_USE_CC2500
#include "libcc2500/inc/TM4C123_CC2500.h"
#endif

#ifdef RF_USE_nRF24L01
#include "libnrf24l01/inc/TM4C123_nRF24L01.h"
#endif

static e_CpuMode g_eCpuMode = CPU_MODE_RUN;

void setCpuMode(e_CpuMode eMode)
{
	g_eCpuMode = eMode;
	IntTrigger(INT_SW_TRIGGER_LPM);
}

e_CpuMode getCpuMode(void)
{
	return g_eCpuMode;
}

void softReset(void)
{
	DEBUG_PRINT("\nReset!\n");

	HWREG(NVIC_APINT) = (NVIC_APINT_VECTKEY |
	NVIC_APINT_SYSRESETREQ);
}

void returnSleep(void)
{
	DEBUG_PRINT("returnSleep()\n");

	IntTrigger(INT_SW_TRIGGER_LPM);
}

void LowPowerModeIntHandler(void) {
	DEBUG_PRINT("Entering LowPowerModeIntHandler()...\n");

	switch (g_eCpuMode) {
	case CPU_MODE_SLEEP:
		gotoSleepMode();
		break;
	case CPU_MODE_DEEP_SLEEP:
		gotoDeepSleepMode();
		break;
	default:	// CPU_MODE_RUN
		break;
	}

	DEBUG_PRINT("Return from LowPowerModeIntHandler()!\n");
}

void initLowPowerMode() {
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
#ifdef RF_USE_CC2500
	ROM_SysCtlPeripheralSleepEnable(CC2500_INT_PORT_CLOCK); // IMPORTANCE: allow IRQ pin of RF module wake CPU up when new byte has received.
#endif
#ifdef RF_USE_nRF24L01
	ROM_SysCtlPeripheralSleepEnable(RF24_INT_PORT_CLOCK); // IMPORTANCE: allow IRQ pin of RF module wake CPU up when new byte has received.
#endif
	ROM_SysCtlPeripheralSleepEnable(DELAY_TIMER_CLOCK_NON_INT);
#ifdef DEBUG_UTILS
	ROM_SysCtlPeripheralSleepEnable(UART_DEBUG_PERIPH);
#endif
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
#ifdef RF_USE_CC2500
	ROM_SysCtlPeripheralDeepSleepEnable(CC2500_INT_PORT_CLOCK);	// IMPORTANCE: allow IRQ pin of RF module wake CPU up when new byte has received.
#endif
#ifdef RF_USE_nRF24L01
	ROM_SysCtlPeripheralDeepSleepEnable(RF24_INT_PORT_CLOCK);	// IMPORTANCE: allow IRQ pin of RF module wake CPU up when new byte has received.
#endif
	ROM_SysCtlPeripheralDeepSleepEnable(DELAY_TIMER_CLOCK_NON_INT);
#ifdef DEBUG_UTILS
	ROM_SysCtlPeripheralDeepSleepEnable(UART_DEBUG_PERIPH);
#endif

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
	ROM_SysCtlPeripheralClockGating(true);

	//==============================================
	// IMPORTANCE: Configure Software Interrupt
	//==============================================
	ROM_IntPrioritySet(INT_SW_TRIGGER_LPM, PRIORITY_LOW_POWER_MODE);
	IntRegister(INT_SW_TRIGGER_LPM, LowPowerModeIntHandler);
	ROM_IntEnable(INT_SW_TRIGGER_LPM);

	g_eCpuMode = CPU_MODE_RUN;

	DEBUG_PRINT("init Low Power Mode: OK, CPU in RUN mode.\n");
}

void gotoSleepMode() {
	DEBUG_PRINT("Entering Sleep mode\n");

	//
	// NOTE: Switch clock to PIOSC and power down the MOSC before going into Sleep.
	// This will be the Run mode's clock configuration after wake up form Sleep mode.
	// So that reconfigure system clock should be considered if required.
	//
	ROM_SysCtlClockSet(SYSCTL_OSC_INT | SYSCTL_USE_OSC | SYSCTL_MAIN_OSC_DIS);

	//disableMOTOR();

	turnOffLED(LED_ALL);
	turnOnLED(LED_GREEN);

	ROM_SysCtlSleep();

	turnOffLED(LED_GREEN);

	DEBUG_PRINT("Exit Sleep mode\n");
}

void gotoDeepSleepMode() {
	DEBUG_PRINT("Entering Deep Sleep mode\n");

	//disableMOTOR();

	turnOffLED(LED_ALL);

	ROM_SysCtlDeepSleep();

	DEBUG_PRINT("Exit Deep Sleep Mode\n");
}

void wakeUpFormLPM() {
	ROM_SysCtlClockSet(
			SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN
					| SYSCTL_XTAL_16MHZ);

	g_eCpuMode = CPU_MODE_RUN;

	ROM_SysCtlDelay(100000);

	DEBUG_PRINT("CPU wakeup!!!\n");
}
