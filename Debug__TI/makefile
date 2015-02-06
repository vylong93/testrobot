################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := D:/ti/ccsv6/tools/compiler/arm_5.1.6

ORDERED_OBJS += \
$(GEN_CMDS__FLAG) \
"./main.obj" \
"./tm4c123gh6pm_startup_ccs.obj" \
"./utils/uartstdio.obj" \
"./librobot/src/robot_analog.obj" \
"./librobot/src/robot_communication.obj" \
"./librobot/src/robot_eeprom.obj" \
"./librobot/src/robot_lpm.obj" \
"./librobot/src/robot_motor.obj" \
"./librobot/src/robot_speaker.obj" \
"./librobot/src/robot_timer_delay.obj" \
"./libprotocol/src/network.obj" \
"./libnrf24l01/src/TM4C123_nRF24L01.obj" \
"./libnrf24l01/src/nRF24L01.obj" \
"./libcustom/src/custom_clock.obj" \
"./libcustom/src/custom_delay.obj" \
"./libcustom/src/custom_i2c.obj" \
"./libcustom/src/custom_led.obj" \
"./libcustom/src/custom_stickTimer.obj" \
"./libcustom/src/custom_uart_debug.obj" \
"./libcc2500/src/TM4C123_CC2500.obj" \
"./libcc2500/src/cc2500.obj" \
"./eMPL/src/inv_mpu.obj" \
"../tm4c123gh6pm.cmd" \
-l"libc.a" \
-l"D:\Program Files\TI\TivaWare_C_Series-2.1.0.12573\driverlib\ccs\Debug\driverlib.lib" \
-l"D:\Program Files\TI\CMSIS-SP-00300-r4p0-00rel0\CMSIS\Lib\CCS\M4\dsplib-cm4f.lib" \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include utils/subdir_vars.mk
-include librobot/src/subdir_vars.mk
-include libprotocol/src/subdir_vars.mk
-include libnrf24l01/src/subdir_vars.mk
-include libcustom/src/subdir_vars.mk
-include libcc2500/src/subdir_vars.mk
-include eMPL/src/subdir_vars.mk
-include subdir_rules.mk
-include utils/subdir_rules.mk
-include librobot/src/subdir_rules.mk
-include libprotocol/src/subdir_rules.mk
-include libnrf24l01/src/subdir_rules.mk
-include libcustom/src/subdir_rules.mk
-include libcc2500/src/subdir_rules.mk
-include eMPL/src/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
SwarmRobotFirmV2.out \

EXE_OUTPUTS__QUOTED += \
"SwarmRobotFirmV2.out" \

BIN_OUTPUTS += \
SwarmRobotFirmV2.hex \

BIN_OUTPUTS__QUOTED += \
"SwarmRobotFirmV2.hex" \


# All Target
all: SwarmRobotFirmV2.out

# Tool invocations
SwarmRobotFirmV2.out: $(OBJS) $(CMD_SRCS) $(LDS_SRCS) $(GEN_CMDS)
	@echo 'Building target: $@'
	@echo 'Invoking: ARM Linker'
	"D:/ti/ccsv6/tools/compiler/arm_5.1.6/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me -g --gcc --define=ccs="ccs" --define=ARM_MATH_CM4 --define=__FPU_PRESENT=1 --define=PART_TM4C123GH6PM --define=TARGET_IS_BLIZZARD_RB1 --define=RF_USE_nRF24L01 --display_error_number --diag_warning=225 --diag_wrap=off -z -m"SwarmRobotFirmV2.map" --heap_size=10240 --stack_size=10240 -i"D:/ti/ccsv6/tools/compiler/arm_5.1.6/lib" -i"D:/ti/ccsv6/tools/compiler/arm_5.1.6/include" --reread_libs --warn_sections --display_error_number --diag_wrap=off --xml_link_info="SwarmRobotFirmV2_linkInfo.xml" --rom_model -o "SwarmRobotFirmV2.out" $(ORDERED_OBJS)
	@echo 'Finished building target: $@'
	@echo ' '
	@$(MAKE) --no-print-directory post-build

SwarmRobotFirmV2.hex: $(EXE_OUTPUTS)
	@echo 'Invoking: ARM Hex Utility'
	"D:/ti/ccsv6/tools/compiler/arm_5.1.6/bin/armhex"  -o "SwarmRobotFirmV2.hex" $(EXE_OUTPUTS__QUOTED)
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXE_OUTPUTS__QUOTED)$(BIN_OUTPUTS__QUOTED)
	-$(RM) "main.pp" "libprotocol\src\network.pp" "eMPL\src\inv_mpu.pp" 
	-$(RM) "tm4c123gh6pm_startup_ccs.pp" "utils\uartstdio.pp" "librobot\src\robot_analog.pp" "librobot\src\robot_communication.pp" "librobot\src\robot_eeprom.pp" "librobot\src\robot_lpm.pp" "librobot\src\robot_motor.pp" "librobot\src\robot_speaker.pp" "librobot\src\robot_timer_delay.pp" "libnrf24l01\src\TM4C123_nRF24L01.pp" "libnrf24l01\src\nRF24L01.pp" "libcustom\src\custom_clock.pp" "libcustom\src\custom_delay.pp" "libcustom\src\custom_i2c.pp" "libcustom\src\custom_led.pp" "libcustom\src\custom_stickTimer.pp" "libcustom\src\custom_uart_debug.pp" "libcc2500\src\TM4C123_CC2500.pp" "libcc2500\src\cc2500.pp" 
	-$(RM) "main.obj" "tm4c123gh6pm_startup_ccs.obj" "utils\uartstdio.obj" "librobot\src\robot_analog.obj" "librobot\src\robot_communication.obj" "librobot\src\robot_eeprom.obj" "librobot\src\robot_lpm.obj" "librobot\src\robot_motor.obj" "librobot\src\robot_speaker.obj" "librobot\src\robot_timer_delay.obj" "libprotocol\src\network.obj" "libnrf24l01\src\TM4C123_nRF24L01.obj" "libnrf24l01\src\nRF24L01.obj" "libcustom\src\custom_clock.obj" "libcustom\src\custom_delay.obj" "libcustom\src\custom_i2c.obj" "libcustom\src\custom_led.obj" "libcustom\src\custom_stickTimer.obj" "libcustom\src\custom_uart_debug.obj" "libcc2500\src\TM4C123_CC2500.obj" "libcc2500\src\cc2500.obj" "eMPL\src\inv_mpu.obj" 
	-@echo 'Finished clean'
	-@echo ' '

post-build:
	-"D:/ti/ccsv6/tools/compiler/gcc-arm-none-eabi-4_7-2013q3/bin/arm-none-eabi-objcopy.exe" -O ihex  SwarmRobotFirmV2.out SwarmRobotFirmV2.hex
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY: post-build

-include ../makefile.targets

