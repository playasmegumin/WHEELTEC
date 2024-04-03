################################################################################
# 自动生成的文件。不要编辑！
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../WHEELTEC/DataScope_DP.c \
../WHEELTEC/KF.c \
../WHEELTEC/MPU6050.c \
../WHEELTEC/ReadDistance.c \
../WHEELTEC/control.c \
../WHEELTEC/filter.c \
../WHEELTEC/inv_mpu.c \
../WHEELTEC/inv_mpu_dmp_motion_driver.c \
../WHEELTEC/show.c 

OBJS += \
./WHEELTEC/DataScope_DP.o \
./WHEELTEC/KF.o \
./WHEELTEC/MPU6050.o \
./WHEELTEC/ReadDistance.o \
./WHEELTEC/control.o \
./WHEELTEC/filter.o \
./WHEELTEC/inv_mpu.o \
./WHEELTEC/inv_mpu_dmp_motion_driver.o \
./WHEELTEC/show.o 

C_DEPS += \
./WHEELTEC/DataScope_DP.d \
./WHEELTEC/KF.d \
./WHEELTEC/MPU6050.d \
./WHEELTEC/ReadDistance.d \
./WHEELTEC/control.d \
./WHEELTEC/filter.d \
./WHEELTEC/inv_mpu.d \
./WHEELTEC/inv_mpu_dmp_motion_driver.d \
./WHEELTEC/show.d 


# Each subdirectory must supply rules for building sources it contributes
WHEELTEC/%.o WHEELTEC/%.su WHEELTEC/%.cyclo: ../WHEELTEC/%.c WHEELTEC/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I"E:/workspace/ucas/2023-2024-2/WHEELTEC/BSPcode/Inc" -I"E:/workspace/ucas/2023-2024-2/WHEELTEC/WHEELTEC/Inc" -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-WHEELTEC

clean-WHEELTEC:
	-$(RM) ./WHEELTEC/DataScope_DP.cyclo ./WHEELTEC/DataScope_DP.d ./WHEELTEC/DataScope_DP.o ./WHEELTEC/DataScope_DP.su ./WHEELTEC/KF.cyclo ./WHEELTEC/KF.d ./WHEELTEC/KF.o ./WHEELTEC/KF.su ./WHEELTEC/MPU6050.cyclo ./WHEELTEC/MPU6050.d ./WHEELTEC/MPU6050.o ./WHEELTEC/MPU6050.su ./WHEELTEC/ReadDistance.cyclo ./WHEELTEC/ReadDistance.d ./WHEELTEC/ReadDistance.o ./WHEELTEC/ReadDistance.su ./WHEELTEC/control.cyclo ./WHEELTEC/control.d ./WHEELTEC/control.o ./WHEELTEC/control.su ./WHEELTEC/filter.cyclo ./WHEELTEC/filter.d ./WHEELTEC/filter.o ./WHEELTEC/filter.su ./WHEELTEC/inv_mpu.cyclo ./WHEELTEC/inv_mpu.d ./WHEELTEC/inv_mpu.o ./WHEELTEC/inv_mpu.su ./WHEELTEC/inv_mpu_dmp_motion_driver.cyclo ./WHEELTEC/inv_mpu_dmp_motion_driver.d ./WHEELTEC/inv_mpu_dmp_motion_driver.o ./WHEELTEC/inv_mpu_dmp_motion_driver.su ./WHEELTEC/show.cyclo ./WHEELTEC/show.d ./WHEELTEC/show.o ./WHEELTEC/show.su

.PHONY: clean-WHEELTEC

