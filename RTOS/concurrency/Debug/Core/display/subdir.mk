################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/display/fonts.c \
../Core/display/ssd1306.c \
../Core/display/test.c 

OBJS += \
./Core/display/fonts.o \
./Core/display/ssd1306.o \
./Core/display/test.o 

C_DEPS += \
./Core/display/fonts.d \
./Core/display/ssd1306.d \
./Core/display/test.d 


# Each subdirectory must supply rules for building sources it contributes
Core/display/%.o Core/display/%.su Core/display/%.cyclo: ../Core/display/%.c Core/display/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-display

clean-Core-2f-display:
	-$(RM) ./Core/display/fonts.cyclo ./Core/display/fonts.d ./Core/display/fonts.o ./Core/display/fonts.su ./Core/display/ssd1306.cyclo ./Core/display/ssd1306.d ./Core/display/ssd1306.o ./Core/display/ssd1306.su ./Core/display/test.cyclo ./Core/display/test.d ./Core/display/test.o ./Core/display/test.su

.PHONY: clean-Core-2f-display

