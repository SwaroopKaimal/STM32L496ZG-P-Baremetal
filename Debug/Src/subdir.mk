################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/005spi_txonly_arduino.c \
../Src/syscalls.c \
../Src/sysmem.c 

OBJS += \
./Src/005spi_txonly_arduino.o \
./Src/syscalls.o \
./Src/sysmem.o 

C_DEPS += \
./Src/005spi_txonly_arduino.d \
./Src/syscalls.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32L4 -DSTM32 -DNUCLEO_L496ZG_P -DSTM32L496ZGTxP -c -I../Inc -I"D:/EmbeddedSystems/Workspace_DriverDevelopment/stm32l4xx_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/005spi_txonly_arduino.cyclo ./Src/005spi_txonly_arduino.d ./Src/005spi_txonly_arduino.o ./Src/005spi_txonly_arduino.su ./Src/syscalls.cyclo ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.cyclo ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su

.PHONY: clean-Src

