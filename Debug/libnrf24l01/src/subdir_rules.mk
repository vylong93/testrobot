################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
libnrf24l01/src/%.o: ../libnrf24l01/src/%.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: GNU Compiler'
	"D:/TI/ccsv6/tools/compiler/gcc-arm-none-eabi-4_7-2013q3/bin/arm-none-eabi-gcc.exe" -c -mcpu=cortex-m4 -mthumb -mfloat-abi=softfp -mfpu=fpv4-sp-d16 -DPART_TM4C123GH6PM -DTARGET_IS_BLIZZARD_RB1 -Dgcc -I"D:/TI/ccsv6/tools/compiler/gcc-arm-none-eabi-4_7-2013q3/arm-none-eabi/include" -I"E:/00 SwarmRobot/Workspace/Bootloader_Test" -I"D:/Program Files/TI/TivaWare_C_Series-2.1.0.12573" -O0 -ffunction-sections -fdata-sections -g -gstrict-dwarf -Wall -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


