################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/AS7343.c \
../Core/Src/PASCO2.c \
../Core/Src/PCA9548a.c \
../Core/Src/VEML6031.c \
../Core/Src/main.c \
../Core/Src/mlx90614.c \
../Core/Src/sht3x.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c 

OBJS += \
./Core/Src/AS7343.o \
./Core/Src/PASCO2.o \
./Core/Src/PCA9548a.o \
./Core/Src/VEML6031.o \
./Core/Src/main.o \
./Core/Src/mlx90614.o \
./Core/Src/sht3x.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o 

C_DEPS += \
./Core/Src/AS7343.d \
./Core/Src/PASCO2.d \
./Core/Src/PCA9548a.d \
./Core/Src/VEML6031.d \
./Core/Src/main.d \
./Core/Src/mlx90614.d \
./Core/Src/sht3x.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/AS7343.d ./Core/Src/AS7343.o ./Core/Src/AS7343.su ./Core/Src/PASCO2.d ./Core/Src/PASCO2.o ./Core/Src/PASCO2.su ./Core/Src/PCA9548a.d ./Core/Src/PCA9548a.o ./Core/Src/PCA9548a.su ./Core/Src/VEML6031.d ./Core/Src/VEML6031.o ./Core/Src/VEML6031.su ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/mlx90614.d ./Core/Src/mlx90614.o ./Core/Src/mlx90614.su ./Core/Src/sht3x.d ./Core/Src/sht3x.o ./Core/Src/sht3x.su ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su

.PHONY: clean-Core-2f-Src

