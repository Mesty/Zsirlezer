################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/adc.c \
../Src/gpio.c \
../Src/main.c \
../Src/spi.c \
../Src/stm32f4xx_hal_msp.c \
../Src/stm32f4xx_it.c \
../Src/usart.c 

OBJS += \
./Src/adc.o \
./Src/gpio.o \
./Src/main.o \
./Src/spi.o \
./Src/stm32f4xx_hal_msp.o \
./Src/stm32f4xx_it.o \
./Src/usart.o 

C_DEPS += \
./Src/adc.d \
./Src/gpio.d \
./Src/main.d \
./Src/spi.d \
./Src/stm32f4xx_hal_msp.d \
./Src/stm32f4xx_it.d \
./Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -DUSE_HAL_DRIVER -DSTM32F446xx -I"E:/adokhumenthumok/bme/1MSC/2_felev/robonAUT/stm32projects/cube_nucleo_spi/nucleo_cube_spi/Inc" -I"E:/adokhumenthumok/bme/1MSC/2_felev/robonAUT/stm32projects/cube_nucleo_spi/nucleo_cube_spi/Drivers/STM32F4xx_HAL_Driver/Inc" -I"E:/adokhumenthumok/bme/1MSC/2_felev/robonAUT/stm32projects/cube_nucleo_spi/nucleo_cube_spi/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"E:/adokhumenthumok/bme/1MSC/2_felev/robonAUT/stm32projects/cube_nucleo_spi/nucleo_cube_spi/Drivers/CMSIS/Include" -I"E:/adokhumenthumok/bme/1MSC/2_felev/robonAUT/stm32projects/cube_nucleo_spi/nucleo_cube_spi/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"E:/adokhumenthumok/bme/1MSC/2_felev/robonAUT/stm32projects/cube_nucleo_spi/nucleo_cube_spi/Inc"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


