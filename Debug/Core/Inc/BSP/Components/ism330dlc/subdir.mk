################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/BSP/Components/ism330dlc/ism330dlc.c \
../Core/Inc/BSP/Components/ism330dlc/ism330dlc_reg.c 

C_DEPS += \
./Core/Inc/BSP/Components/ism330dlc/ism330dlc.d \
./Core/Inc/BSP/Components/ism330dlc/ism330dlc_reg.d 

OBJS += \
./Core/Inc/BSP/Components/ism330dlc/ism330dlc.o \
./Core/Inc/BSP/Components/ism330dlc/ism330dlc_reg.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/BSP/Components/ism330dlc/%.o Core/Inc/BSP/Components/ism330dlc/%.su: ../Core/Inc/BSP/Components/ism330dlc/%.c Core/Inc/BSP/Components/ism330dlc/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DUSE_HAL_DRIVER -DSTM32U585xx -DTX_INCLUDE_USER_DEFINE_FILE -DTX_SINGLE_MODE_NON_SECURE=1 -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -I../AZURE_RTOS/App -I../Middlewares/ST/threadx/common/inc -I../Middlewares/ST/threadx/ports/cortex_m33/gnu/inc -I../Middlewares/ST/threadx/utility/low_power -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-BSP-2f-Components-2f-ism330dlc

clean-Core-2f-Inc-2f-BSP-2f-Components-2f-ism330dlc:
	-$(RM) ./Core/Inc/BSP/Components/ism330dlc/ism330dlc.d ./Core/Inc/BSP/Components/ism330dlc/ism330dlc.o ./Core/Inc/BSP/Components/ism330dlc/ism330dlc.su ./Core/Inc/BSP/Components/ism330dlc/ism330dlc_reg.d ./Core/Inc/BSP/Components/ism330dlc/ism330dlc_reg.o ./Core/Inc/BSP/Components/ism330dlc/ism330dlc_reg.su

.PHONY: clean-Core-2f-Inc-2f-BSP-2f-Components-2f-ism330dlc

