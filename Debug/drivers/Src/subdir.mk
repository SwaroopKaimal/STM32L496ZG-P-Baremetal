################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/Src/stm32l496xx_gpio.c \
../drivers/Src/stm32l496xx_spi.c 

OBJS += \
./drivers/Src/stm32l496xx_gpio.o \
./drivers/Src/stm32l496xx_spi.o 

C_DEPS += \
./drivers/Src/stm32l496xx_gpio.d \
./drivers/Src/stm32l496xx_spi.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/Src/%.o drivers/Src/%.su drivers/Src/%.cyclo: ../drivers/Src/%.c drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32L4 -DSTM32 -DNUCLEO_L496ZG_P -DSTM32L496ZGTxP -c -I../Inc -I"D:/EmbeddedSystems/Workspace_DriverDevelopment/stm32l4xx_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-drivers-2f-Src

clean-drivers-2f-Src:
	-$(RM) ./drivers/Src/stm32l496xx_gpio.cyclo ./drivers/Src/stm32l496xx_gpio.d ./drivers/Src/stm32l496xx_gpio.o ./drivers/Src/stm32l496xx_gpio.su ./drivers/Src/stm32l496xx_spi.cyclo ./drivers/Src/stm32l496xx_spi.d ./drivers/Src/stm32l496xx_spi.o ./drivers/Src/stm32l496xx_spi.su

.PHONY: clean-drivers-2f-Src

