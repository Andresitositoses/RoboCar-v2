################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/BSP/motion_sensors/b_u585i_iot02a_motion_sensors.c 

C_DEPS += \
./Core/Inc/BSP/motion_sensors/b_u585i_iot02a_motion_sensors.d 

OBJS += \
./Core/Inc/BSP/motion_sensors/b_u585i_iot02a_motion_sensors.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/BSP/motion_sensors/%.o Core/Inc/BSP/motion_sensors/%.su: ../Core/Inc/BSP/motion_sensors/%.c Core/Inc/BSP/motion_sensors/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DUSE_HAL_DRIVER -DSTM32U585xx -DTX_INCLUDE_USER_DEFINE_FILE -DTX_SINGLE_MODE_NON_SECURE=1 -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -I../AZURE_RTOS/App -I../Middlewares/ST/threadx/common/inc -I../Middlewares/ST/threadx/ports/cortex_m33/gnu/inc -I../Middlewares/ST/threadx/utility/low_power -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-BSP-2f-motion_sensors

clean-Core-2f-Inc-2f-BSP-2f-motion_sensors:
	-$(RM) ./Core/Inc/BSP/motion_sensors/b_u585i_iot02a_motion_sensors.d ./Core/Inc/BSP/motion_sensors/b_u585i_iot02a_motion_sensors.o ./Core/Inc/BSP/motion_sensors/b_u585i_iot02a_motion_sensors.su

.PHONY: clean-Core-2f-Inc-2f-BSP-2f-motion_sensors

