################################################################################
# 自动生成的文件。不要编辑！
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../BSPcode/LED.c \
../BSPcode/bsp_adc.c \
../BSPcode/delay.c \
../BSPcode/encoder.c \
../BSPcode/iic.c \
../BSPcode/key.c \
../BSPcode/oled.c \
../BSPcode/usart3.c 

OBJS += \
./BSPcode/LED.o \
./BSPcode/bsp_adc.o \
./BSPcode/delay.o \
./BSPcode/encoder.o \
./BSPcode/iic.o \
./BSPcode/key.o \
./BSPcode/oled.o \
./BSPcode/usart3.o 

C_DEPS += \
./BSPcode/LED.d \
./BSPcode/bsp_adc.d \
./BSPcode/delay.d \
./BSPcode/encoder.d \
./BSPcode/iic.d \
./BSPcode/key.d \
./BSPcode/oled.d \
./BSPcode/usart3.d 


# Each subdirectory must supply rules for building sources it contributes
BSPcode/%.o BSPcode/%.su BSPcode/%.cyclo: ../BSPcode/%.c BSPcode/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I"E:/workspace/ucas/2023-2024-2/WHEELTEC/BSPcode/Inc" -I"E:/workspace/ucas/2023-2024-2/WHEELTEC/WHEELTEC/Inc" -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-BSPcode

clean-BSPcode:
	-$(RM) ./BSPcode/LED.cyclo ./BSPcode/LED.d ./BSPcode/LED.o ./BSPcode/LED.su ./BSPcode/bsp_adc.cyclo ./BSPcode/bsp_adc.d ./BSPcode/bsp_adc.o ./BSPcode/bsp_adc.su ./BSPcode/delay.cyclo ./BSPcode/delay.d ./BSPcode/delay.o ./BSPcode/delay.su ./BSPcode/encoder.cyclo ./BSPcode/encoder.d ./BSPcode/encoder.o ./BSPcode/encoder.su ./BSPcode/iic.cyclo ./BSPcode/iic.d ./BSPcode/iic.o ./BSPcode/iic.su ./BSPcode/key.cyclo ./BSPcode/key.d ./BSPcode/key.o ./BSPcode/key.su ./BSPcode/oled.cyclo ./BSPcode/oled.d ./BSPcode/oled.o ./BSPcode/oled.su ./BSPcode/usart3.cyclo ./BSPcode/usart3.d ./BSPcode/usart3.o ./BSPcode/usart3.su

.PHONY: clean-BSPcode

