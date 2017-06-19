################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/dht11.c \
../Src/display_manager.c \
../Src/main.c \
../Src/pmsensor.c \
../Src/stm32l4xx_hal_msp.c \
../Src/stm32l4xx_it.c \
../Src/system_stm32l4xx.c 

OBJS += \
./Src/dht11.o \
./Src/display_manager.o \
./Src/main.o \
./Src/pmsensor.o \
./Src/stm32l4xx_hal_msp.o \
./Src/stm32l4xx_it.o \
./Src/system_stm32l4xx.o 

C_DEPS += \
./Src/dht11.d \
./Src/display_manager.d \
./Src/main.d \
./Src/pmsensor.d \
./Src/stm32l4xx_hal_msp.d \
./Src/stm32l4xx_it.d \
./Src/system_stm32l4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32L476xx -I"G:/lastchance/maybethistimemythesis/Inc" -I"G:/lastchance/maybethistimemythesis/Drivers/STM32L4xx_HAL_Driver/Inc" -I"G:/lastchance/maybethistimemythesis/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy" -I"G:/lastchance/maybethistimemythesis/Drivers/CMSIS/Device/ST/STM32L4xx/Include" -I"G:/lastchance/maybethistimemythesis/Drivers/CMSIS/Include" -I"G:/lastchance/maybethistimemythesis/Drivers/STM32L476G-Discovery"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


