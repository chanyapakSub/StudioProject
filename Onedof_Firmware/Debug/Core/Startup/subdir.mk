################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32g474retx.s 

OBJS += \
./Core/Startup/startup_stm32g474retx.o 

S_DEPS += \
./Core/Startup/startup_stm32g474retx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/%.o: ../Core/Startup/%.s Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -I"D:/HomeWork/FRA262/Firmware/StudioProject/Onedof_Firmware/Source/BasicMathFunctions" -I"D:/HomeWork/FRA262/Firmware/StudioProject/Onedof_Firmware/Source/BayesFunctions" -I"D:/HomeWork/FRA262/Firmware/StudioProject/Onedof_Firmware/Source/CommonTables" -I"D:/HomeWork/FRA262/Firmware/StudioProject/Onedof_Firmware/Source/ComplexMathFunctions" -I"D:/HomeWork/FRA262/Firmware/StudioProject/Onedof_Firmware/Source/ControllerFunctions" -I"D:/HomeWork/FRA262/Firmware/StudioProject/Onedof_Firmware/Source/DistanceFunctions" -I"D:/HomeWork/FRA262/Firmware/StudioProject/Onedof_Firmware/Source/FastMathFunctions" -I"D:/HomeWork/FRA262/Firmware/StudioProject/Onedof_Firmware/Source/FilteringFunctions" -I"D:/HomeWork/FRA262/Firmware/StudioProject/Onedof_Firmware/Source/InterpolationFunctions" -I"D:/HomeWork/FRA262/Firmware/StudioProject/Onedof_Firmware/Source/MatrixFunctions" -I"D:/HomeWork/FRA262/Firmware/StudioProject/Onedof_Firmware/Source/QuaternionMathFunctions" -I"D:/HomeWork/FRA262/Firmware/StudioProject/Onedof_Firmware/Source/StatisticsFunctions" -I"D:/HomeWork/FRA262/Firmware/StudioProject/Onedof_Firmware/Source/SupportFunctions" -I"D:/HomeWork/FRA262/Firmware/StudioProject/Onedof_Firmware/Source/SVMFunctions" -I"D:/HomeWork/FRA262/Firmware/StudioProject/Onedof_Firmware/Source/TransformFunctions" -I"D:/HomeWork/FRA262/Firmware/StudioProject/Onedof_Firmware/Source/WindowFunctions" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Core-2f-Startup

clean-Core-2f-Startup:
	-$(RM) ./Core/Startup/startup_stm32g474retx.d ./Core/Startup/startup_stm32g474retx.o

.PHONY: clean-Core-2f-Startup

