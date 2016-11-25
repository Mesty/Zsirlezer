################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/BME/RobonAUT/Zsirlezer/Zsirlezer_SW/Src/main.c \
C:/BME/RobonAUT/Zsirlezer/Zsirlezer_SW/Src/stm32f4xx_hal_msp.c \
C:/BME/RobonAUT/Zsirlezer/Zsirlezer_SW/Src/stm32f4xx_it.c 

OBJS += \
./Application/User/main.o \
./Application/User/stm32f4xx_hal_msp.o \
./Application/User/stm32f4xx_it.o 

C_DEPS += \
./Application/User/main.d \
./Application/User/stm32f4xx_hal_msp.d \
./Application/User/stm32f4xx_it.d 


# Each subdirectory must supply rules for building sources it contributes
Application/User/main.o: C:/BME/RobonAUT/Zsirlezer/Zsirlezer_SW/Src/main.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -DUSE_HAL_DRIVER -DSTM32F446xx -I"C:/BME/RobonAUT/Zsirlezer/Zsirlezer_SW/Inc" -I"C:/BME/RobonAUT/Zsirlezer/Zsirlezer_SW/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/BME/RobonAUT/Zsirlezer/Zsirlezer_SW/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"C:/BME/RobonAUT/Zsirlezer/Zsirlezer_SW/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/BME/RobonAUT/Zsirlezer/Zsirlezer_SW/Drivers/CMSIS/Include" -I"C:/BME/RobonAUT/Zsirlezer/Zsirlezer_SW/Inc"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Application/User/stm32f4xx_hal_msp.o: C:/BME/RobonAUT/Zsirlezer/Zsirlezer_SW/Src/stm32f4xx_hal_msp.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -DUSE_HAL_DRIVER -DSTM32F446xx -I"C:/BME/RobonAUT/Zsirlezer/Zsirlezer_SW/Inc" -I"C:/BME/RobonAUT/Zsirlezer/Zsirlezer_SW/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/BME/RobonAUT/Zsirlezer/Zsirlezer_SW/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"C:/BME/RobonAUT/Zsirlezer/Zsirlezer_SW/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/BME/RobonAUT/Zsirlezer/Zsirlezer_SW/Drivers/CMSIS/Include" -I"C:/BME/RobonAUT/Zsirlezer/Zsirlezer_SW/Inc"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Application/User/stm32f4xx_it.o: C:/BME/RobonAUT/Zsirlezer/Zsirlezer_SW/Src/stm32f4xx_it.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -DUSE_HAL_DRIVER -DSTM32F446xx -I"C:/BME/RobonAUT/Zsirlezer/Zsirlezer_SW/Inc" -I"C:/BME/RobonAUT/Zsirlezer/Zsirlezer_SW/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/BME/RobonAUT/Zsirlezer/Zsirlezer_SW/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"C:/BME/RobonAUT/Zsirlezer/Zsirlezer_SW/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/BME/RobonAUT/Zsirlezer/Zsirlezer_SW/Drivers/CMSIS/Include" -I"C:/BME/RobonAUT/Zsirlezer/Zsirlezer_SW/Inc"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


