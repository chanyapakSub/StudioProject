################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Basesystem.c \
../Core/Src/ModBusRTU.c \
../Core/Src/Scurve.c \
../Core/Src/Trajectory.c \
../Core/Src/Trapezoidal.c \
../Core/Src/adc.c \
../Core/Src/eff.c \
../Core/Src/joy.c \
../Core/Src/kalman.c \
../Core/Src/kalman_saifah.c \
../Core/Src/lowpass.c \
../Core/Src/main.c \
../Core/Src/pid.c \
../Core/Src/pidNing.c \
../Core/Src/pwm.c \
../Core/Src/qei.c \
../Core/Src/state.c \
../Core/Src/stm32g4xx_hal_msp.c \
../Core/Src/stm32g4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32g4xx.c 

OBJS += \
./Core/Src/Basesystem.o \
./Core/Src/ModBusRTU.o \
./Core/Src/Scurve.o \
./Core/Src/Trajectory.o \
./Core/Src/Trapezoidal.o \
./Core/Src/adc.o \
./Core/Src/eff.o \
./Core/Src/joy.o \
./Core/Src/kalman.o \
./Core/Src/kalman_saifah.o \
./Core/Src/lowpass.o \
./Core/Src/main.o \
./Core/Src/pid.o \
./Core/Src/pidNing.o \
./Core/Src/pwm.o \
./Core/Src/qei.o \
./Core/Src/state.o \
./Core/Src/stm32g4xx_hal_msp.o \
./Core/Src/stm32g4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32g4xx.o 

C_DEPS += \
./Core/Src/Basesystem.d \
./Core/Src/ModBusRTU.d \
./Core/Src/Scurve.d \
./Core/Src/Trajectory.d \
./Core/Src/Trapezoidal.d \
./Core/Src/adc.d \
./Core/Src/eff.d \
./Core/Src/joy.d \
./Core/Src/kalman.d \
./Core/Src/kalman_saifah.d \
./Core/Src/lowpass.d \
./Core/Src/main.d \
./Core/Src/pid.d \
./Core/Src/pidNing.d \
./Core/Src/pwm.d \
./Core/Src/qei.d \
./Core/Src/state.d \
./Core/Src/stm32g4xx_hal_msp.d \
./Core/Src/stm32g4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32g4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I"D:/HomeWork/FRA262/Firmware/StudioProject/Onedof_Firmware/Source/BasicMathFunctions" -I"D:/HomeWork/FRA262/Firmware/StudioProject/Onedof_Firmware/Source/BayesFunctions" -I"D:/HomeWork/FRA262/Firmware/StudioProject/Onedof_Firmware/Source/CommonTables" -I"D:/HomeWork/FRA262/Firmware/StudioProject/Onedof_Firmware/Source/ComplexMathFunctions" -I"D:/HomeWork/FRA262/Firmware/StudioProject/Onedof_Firmware/Source/ControllerFunctions" -I"D:/HomeWork/FRA262/Firmware/StudioProject/Onedof_Firmware/Source/DistanceFunctions" -I"D:/HomeWork/FRA262/Firmware/StudioProject/Onedof_Firmware/Source/FastMathFunctions" -I"D:/HomeWork/FRA262/Firmware/StudioProject/Onedof_Firmware/Source/FilteringFunctions" -I"D:/HomeWork/FRA262/Firmware/StudioProject/Onedof_Firmware/Source/InterpolationFunctions" -I"D:/HomeWork/FRA262/Firmware/StudioProject/Onedof_Firmware/Source/MatrixFunctions" -I"D:/HomeWork/FRA262/Firmware/StudioProject/Onedof_Firmware/Source/QuaternionMathFunctions" -I"D:/HomeWork/FRA262/Firmware/StudioProject/Onedof_Firmware/Source/StatisticsFunctions" -I"D:/HomeWork/FRA262/Firmware/StudioProject/Onedof_Firmware/Source/SupportFunctions" -I"D:/HomeWork/FRA262/Firmware/StudioProject/Onedof_Firmware/Source/SVMFunctions" -I"D:/HomeWork/FRA262/Firmware/StudioProject/Onedof_Firmware/Source/TransformFunctions" -I"D:/HomeWork/FRA262/Firmware/StudioProject/Onedof_Firmware/Source/WindowFunctions" -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/Core/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/PrivateInclude/ -I../Middlewares/Third_Party/ARM_CMSIS/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/Basesystem.cyclo ./Core/Src/Basesystem.d ./Core/Src/Basesystem.o ./Core/Src/Basesystem.su ./Core/Src/ModBusRTU.cyclo ./Core/Src/ModBusRTU.d ./Core/Src/ModBusRTU.o ./Core/Src/ModBusRTU.su ./Core/Src/Scurve.cyclo ./Core/Src/Scurve.d ./Core/Src/Scurve.o ./Core/Src/Scurve.su ./Core/Src/Trajectory.cyclo ./Core/Src/Trajectory.d ./Core/Src/Trajectory.o ./Core/Src/Trajectory.su ./Core/Src/Trapezoidal.cyclo ./Core/Src/Trapezoidal.d ./Core/Src/Trapezoidal.o ./Core/Src/Trapezoidal.su ./Core/Src/adc.cyclo ./Core/Src/adc.d ./Core/Src/adc.o ./Core/Src/adc.su ./Core/Src/eff.cyclo ./Core/Src/eff.d ./Core/Src/eff.o ./Core/Src/eff.su ./Core/Src/joy.cyclo ./Core/Src/joy.d ./Core/Src/joy.o ./Core/Src/joy.su ./Core/Src/kalman.cyclo ./Core/Src/kalman.d ./Core/Src/kalman.o ./Core/Src/kalman.su ./Core/Src/kalman_saifah.cyclo ./Core/Src/kalman_saifah.d ./Core/Src/kalman_saifah.o ./Core/Src/kalman_saifah.su ./Core/Src/lowpass.cyclo ./Core/Src/lowpass.d ./Core/Src/lowpass.o ./Core/Src/lowpass.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/pid.cyclo ./Core/Src/pid.d ./Core/Src/pid.o ./Core/Src/pid.su ./Core/Src/pidNing.cyclo ./Core/Src/pidNing.d ./Core/Src/pidNing.o ./Core/Src/pidNing.su ./Core/Src/pwm.cyclo ./Core/Src/pwm.d ./Core/Src/pwm.o ./Core/Src/pwm.su ./Core/Src/qei.cyclo ./Core/Src/qei.d ./Core/Src/qei.o ./Core/Src/qei.su ./Core/Src/state.cyclo ./Core/Src/state.d ./Core/Src/state.o ./Core/Src/state.su ./Core/Src/stm32g4xx_hal_msp.cyclo ./Core/Src/stm32g4xx_hal_msp.d ./Core/Src/stm32g4xx_hal_msp.o ./Core/Src/stm32g4xx_hal_msp.su ./Core/Src/stm32g4xx_it.cyclo ./Core/Src/stm32g4xx_it.d ./Core/Src/stm32g4xx_it.o ./Core/Src/stm32g4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32g4xx.cyclo ./Core/Src/system_stm32g4xx.d ./Core/Src/system_stm32g4xx.o ./Core/Src/system_stm32g4xx.su

.PHONY: clean-Core-2f-Src

