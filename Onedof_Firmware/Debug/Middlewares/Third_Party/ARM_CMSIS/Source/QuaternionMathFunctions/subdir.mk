################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/ARM_CMSIS/Source/QuaternionMathFunctions/QuaternionMathFunctions.c 

OBJS += \
./Middlewares/Third_Party/ARM_CMSIS/Source/QuaternionMathFunctions/QuaternionMathFunctions.o 

C_DEPS += \
./Middlewares/Third_Party/ARM_CMSIS/Source/QuaternionMathFunctions/QuaternionMathFunctions.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/ARM_CMSIS/Source/QuaternionMathFunctions/%.o Middlewares/Third_Party/ARM_CMSIS/Source/QuaternionMathFunctions/%.su Middlewares/Third_Party/ARM_CMSIS/Source/QuaternionMathFunctions/%.cyclo: ../Middlewares/Third_Party/ARM_CMSIS/Source/QuaternionMathFunctions/%.c Middlewares/Third_Party/ARM_CMSIS/Source/QuaternionMathFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I"D:/HomeWork/FRA262/Firmware/StudioProject/Onedof_Firmware/Source/BasicMathFunctions" -I"D:/HomeWork/FRA262/Firmware/StudioProject/Onedof_Firmware/Source/BayesFunctions" -I"D:/HomeWork/FRA262/Firmware/StudioProject/Onedof_Firmware/Source/CommonTables" -I"D:/HomeWork/FRA262/Firmware/StudioProject/Onedof_Firmware/Source/ComplexMathFunctions" -I"D:/HomeWork/FRA262/Firmware/StudioProject/Onedof_Firmware/Source/ControllerFunctions" -I"D:/HomeWork/FRA262/Firmware/StudioProject/Onedof_Firmware/Source/DistanceFunctions" -I"D:/HomeWork/FRA262/Firmware/StudioProject/Onedof_Firmware/Source/FastMathFunctions" -I"D:/HomeWork/FRA262/Firmware/StudioProject/Onedof_Firmware/Source/FilteringFunctions" -I"D:/HomeWork/FRA262/Firmware/StudioProject/Onedof_Firmware/Source/InterpolationFunctions" -I"D:/HomeWork/FRA262/Firmware/StudioProject/Onedof_Firmware/Source/MatrixFunctions" -I"D:/HomeWork/FRA262/Firmware/StudioProject/Onedof_Firmware/Source/QuaternionMathFunctions" -I"D:/HomeWork/FRA262/Firmware/StudioProject/Onedof_Firmware/Source/StatisticsFunctions" -I"D:/HomeWork/FRA262/Firmware/StudioProject/Onedof_Firmware/Source/SupportFunctions" -I"D:/HomeWork/FRA262/Firmware/StudioProject/Onedof_Firmware/Source/SVMFunctions" -I"D:/HomeWork/FRA262/Firmware/StudioProject/Onedof_Firmware/Source/TransformFunctions" -I"D:/HomeWork/FRA262/Firmware/StudioProject/Onedof_Firmware/Source/WindowFunctions" -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/Core/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/PrivateInclude/ -I../Middlewares/Third_Party/ARM_CMSIS/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-Third_Party-2f-ARM_CMSIS-2f-Source-2f-QuaternionMathFunctions

clean-Middlewares-2f-Third_Party-2f-ARM_CMSIS-2f-Source-2f-QuaternionMathFunctions:
	-$(RM) ./Middlewares/Third_Party/ARM_CMSIS/Source/QuaternionMathFunctions/QuaternionMathFunctions.cyclo ./Middlewares/Third_Party/ARM_CMSIS/Source/QuaternionMathFunctions/QuaternionMathFunctions.d ./Middlewares/Third_Party/ARM_CMSIS/Source/QuaternionMathFunctions/QuaternionMathFunctions.o ./Middlewares/Third_Party/ARM_CMSIS/Source/QuaternionMathFunctions/QuaternionMathFunctions.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-ARM_CMSIS-2f-Source-2f-QuaternionMathFunctions

