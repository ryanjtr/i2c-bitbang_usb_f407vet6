################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../i2c_emulator/i2c_bitbang.c 

OBJS += \
./i2c_emulator/i2c_bitbang.o 

C_DEPS += \
./i2c_emulator/i2c_bitbang.d 


# Each subdirectory must supply rules for building sources it contributes
i2c_emulator/%.o i2c_emulator/%.su i2c_emulator/%.cyclo: ../i2c_emulator/%.c i2c_emulator/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -DUSE_FULL_LL_DRIVER -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I"F:/STM32project/modelization/test_i2c_usb_f407vet6/my_uart" -I"F:/STM32project/modelization/test_i2c_usb_f407vet6/i2c_emulator" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-i2c_emulator

clean-i2c_emulator:
	-$(RM) ./i2c_emulator/i2c_bitbang.cyclo ./i2c_emulator/i2c_bitbang.d ./i2c_emulator/i2c_bitbang.o ./i2c_emulator/i2c_bitbang.su

.PHONY: clean-i2c_emulator

