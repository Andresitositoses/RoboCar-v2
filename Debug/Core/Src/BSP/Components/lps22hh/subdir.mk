################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/BSP/Components/lps22hh/lps22hh.c \
../Core/Src/BSP/Components/lps22hh/lps22hh_reg.c 

C_DEPS += \
./Core/Src/BSP/Components/lps22hh/lps22hh.d \
./Core/Src/BSP/Components/lps22hh/lps22hh_reg.d 

OBJS += \
./Core/Src/BSP/Components/lps22hh/lps22hh.o \
./Core/Src/BSP/Components/lps22hh/lps22hh_reg.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/BSP/Components/lps22hh/%.o Core/Src/BSP/Components/lps22hh/%.su: ../Core/Src/BSP/Components/lps22hh/%.c Core/Src/BSP/Components/lps22hh/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DUSE_HAL_DRIVER -DSTM32U585xx -DTX_INCLUDE_USER_DEFINE_FILE -DTX_SINGLE_MODE_NON_SECURE=1 -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -I../AZURE_RTOS/App -I../Middlewares/ST/threadx/common/inc -I../Middlewares/ST/threadx/ports/cortex_m33/gnu/inc -I../Middlewares/ST/threadx/utility/low_power -I../Core/Inc/X-CUBE-MEMS1 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-BSP-2f-Components-2f-lps22hh

clean-Core-2f-Src-2f-BSP-2f-Components-2f-lps22hh:
	-$(RM) ./Core/Src/BSP/Components/lps22hh/lps22hh.d ./Core/Src/BSP/Components/lps22hh/lps22hh.o ./Core/Src/BSP/Components/lps22hh/lps22hh.su ./Core/Src/BSP/Components/lps22hh/lps22hh_reg.d ./Core/Src/BSP/Components/lps22hh/lps22hh_reg.o ./Core/Src/BSP/Components/lps22hh/lps22hh_reg.su

.PHONY: clean-Core-2f-Src-2f-BSP-2f-Components-2f-lps22hh

