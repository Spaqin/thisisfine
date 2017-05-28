################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/STM32L476G-Discovery/stm32l476g_discovery.c \
../Drivers/STM32L476G-Discovery/stm32l476g_discovery_glass_lcd.c 

OBJS += \
./Drivers/STM32L476G-Discovery/stm32l476g_discovery.o \
./Drivers/STM32L476G-Discovery/stm32l476g_discovery_glass_lcd.o 

C_DEPS += \
./Drivers/STM32L476G-Discovery/stm32l476g_discovery.d \
./Drivers/STM32L476G-Discovery/stm32l476g_discovery_glass_lcd.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/STM32L476G-Discovery/%.o: ../Drivers/STM32L476G-Discovery/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32L476xx -I"G:/lastchance/maybethistimemythesis/Inc" -I"G:/lastchance/maybethistimemythesis/Drivers/STM32L4xx_HAL_Driver/Inc" -I"G:/lastchance/maybethistimemythesis/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy" -I"G:/lastchance/maybethistimemythesis/Drivers/CMSIS/Device/ST/STM32L4xx/Include" -I"G:/lastchance/maybethistimemythesis/Drivers/CMSIS/Include" -I"G:/lastchance/maybethistimemythesis/Drivers/STM32L476G-Discovery"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


