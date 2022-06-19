################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/PinsLib/gpio.cpp \
../Core/Src/PinsLib/pwm.cpp 

OBJS += \
./Core/Src/PinsLib/gpio.o \
./Core/Src/PinsLib/pwm.o 

CPP_DEPS += \
./Core/Src/PinsLib/gpio.d \
./Core/Src/PinsLib/pwm.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/PinsLib/%.o Core/Src/PinsLib/%.su: ../Core/Src/PinsLib/%.cpp Core/Src/PinsLib/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m33 -std=gnu++14 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DUSE_HAL_DRIVER -DSTM32U585xx -DTX_INCLUDE_USER_DEFINE_FILE -DTX_SINGLE_MODE_NON_SECURE=1 -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -I../AZURE_RTOS/App -I../Middlewares/ST/threadx/common/inc -I../Middlewares/ST/threadx/ports/cortex_m33/gnu/inc -I../Middlewares/ST/threadx/utility/low_power -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-PinsLib

clean-Core-2f-Src-2f-PinsLib:
	-$(RM) ./Core/Src/PinsLib/gpio.d ./Core/Src/PinsLib/gpio.o ./Core/Src/PinsLib/gpio.su ./Core/Src/PinsLib/pwm.d ./Core/Src/PinsLib/pwm.o ./Core/Src/PinsLib/pwm.su

.PHONY: clean-Core-2f-Src-2f-PinsLib

