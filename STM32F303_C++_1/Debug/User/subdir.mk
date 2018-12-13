################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../User/LedBlink.cpp \
../User/wrapper.cpp 

OBJS += \
./User/LedBlink.o \
./User/wrapper.o 

CPP_DEPS += \
./User/LedBlink.d \
./User/wrapper.d 


# Each subdirectory must supply rules for building sources it contributes
User/%.o: ../User/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: MCU G++ Compiler'
	@echo %cd%
	arm-none-eabi-g++ -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -D__weak=__attribute__((weak)) -D__packed=__attribute__((__packed__)) -DUSE_HAL_DRIVER -DSTM32F303x8 -I"C:/Users/stern/workspace/STM32F303_C++_1/Inc" -I"C:/Users/stern/workspace/STM32F303_C++_1/Drivers/STM32F3xx_HAL_Driver/Inc" -I"C:/Users/stern/workspace/STM32F303_C++_1/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy" -I"C:/Users/stern/workspace/STM32F303_C++_1/Drivers/CMSIS/Device/ST/STM32F3xx/Include" -I"C:/Users/stern/workspace/STM32F303_C++_1/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fno-exceptions -fno-rtti -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


