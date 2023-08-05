################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/sht21/sht21_driver.c 

OBJS += \
./Drivers/BSP/sht21/sht21_driver.o 

C_DEPS += \
./Drivers/BSP/sht21/sht21_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/sht21/%.o Drivers/BSP/sht21/%.su Drivers/BSP/sht21/%.cyclo: ../Drivers/BSP/sht21/%.c Drivers/BSP/sht21/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L496xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"/home/dnk066/chirag/STM32/work space/I2C/I2C_SHT21_SENSOR/Drivers/BSP/sht21" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-sht21

clean-Drivers-2f-BSP-2f-sht21:
	-$(RM) ./Drivers/BSP/sht21/sht21_driver.cyclo ./Drivers/BSP/sht21/sht21_driver.d ./Drivers/BSP/sht21/sht21_driver.o ./Drivers/BSP/sht21/sht21_driver.su

.PHONY: clean-Drivers-2f-BSP-2f-sht21

