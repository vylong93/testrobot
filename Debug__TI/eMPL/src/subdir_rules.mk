################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
eMPL/src/inv_mpu.obj: ../eMPL/src/inv_mpu.cpp $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"D:/ti/ccsv6/tools/compiler/arm_5.1.6/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me --include_path="D:/ti/ccsv6/tools/compiler/arm_5.1.6/include" --include_path="E:/00 SwarmRobot/Workspace/SVN/RobotTesting/branches/oop_ver" --include_path="D:/Program Files/TI/TivaWare_C_Series-2.1.0.12573" --include_path="D:/Program Files/TI/CMSIS-SP-00300-r4p0-00rel0/CMSIS/Include" -g --gcc --define=ccs="ccs" --define=ARM_MATH_CM4 --define=__FPU_PRESENT=1 --define=PART_TM4C123GH6PM --define=TARGET_IS_BLIZZARD_RB1 --define=RF_USE_nRF24L01 --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="eMPL/src/inv_mpu.pp" --obj_directory="eMPL/src" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


