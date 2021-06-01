################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Inc/Src/GPIOHandlet.cpp \
../Inc/Src/tomtom.cpp 

OBJS += \
./Inc/Src/GPIOHandlet.o \
./Inc/Src/tomtom.o 

CPP_DEPS += \
./Inc/Src/GPIOHandlet.d \
./Inc/Src/tomtom.d 


# Each subdirectory must supply rules for building sources it contributes
Inc/Src/GPIOHandlet.o: ../Inc/Src/GPIOHandlet.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DDEBUG -DNUCLEO_F446RE -c -I../Inc -I"/home/ark/STM32CubeIDE/workspace_1.5.0/driver_cpp/Inc/Inc" -I"/home/ark/STM32CubeIDE/workspace_1.5.0/driver_cpp/Inc/Src" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"Inc/Src/GPIOHandlet.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Inc/Src/tomtom.o: ../Inc/Src/tomtom.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DDEBUG -DNUCLEO_F446RE -c -I../Inc -I"/home/ark/STM32CubeIDE/workspace_1.5.0/driver_cpp/Inc/Inc" -I"/home/ark/STM32CubeIDE/workspace_1.5.0/driver_cpp/Inc/Src" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"Inc/Src/tomtom.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

