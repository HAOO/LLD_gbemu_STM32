################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/bus.c \
../Core/Src/cart.c \
../Core/Src/cpu.c \
../Core/Src/cpu_fetch.c \
../Core/Src/cpu_proc.c \
../Core/Src/cpu_util.c \
../Core/Src/dbg.c \
../Core/Src/dma.c \
../Core/Src/emu.c \
../Core/Src/freertos.c \
../Core/Src/gamepad.c \
../Core/Src/instructions.c \
../Core/Src/interrupts.c \
../Core/Src/io.c \
../Core/Src/lcd.c \
../Core/Src/main.c \
../Core/Src/ppu.c \
../Core/Src/ppu_pipeline.c \
../Core/Src/ppu_sm.c \
../Core/Src/ram.c \
../Core/Src/stack.c \
../Core/Src/stm32f7xx_hal_msp.c \
../Core/Src/stm32f7xx_hal_timebase_tim.c \
../Core/Src/stm32f7xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f7xx.c \
../Core/Src/timer.c \
../Core/Src/ui.c 

OBJS += \
./Core/Src/bus.o \
./Core/Src/cart.o \
./Core/Src/cpu.o \
./Core/Src/cpu_fetch.o \
./Core/Src/cpu_proc.o \
./Core/Src/cpu_util.o \
./Core/Src/dbg.o \
./Core/Src/dma.o \
./Core/Src/emu.o \
./Core/Src/freertos.o \
./Core/Src/gamepad.o \
./Core/Src/instructions.o \
./Core/Src/interrupts.o \
./Core/Src/io.o \
./Core/Src/lcd.o \
./Core/Src/main.o \
./Core/Src/ppu.o \
./Core/Src/ppu_pipeline.o \
./Core/Src/ppu_sm.o \
./Core/Src/ram.o \
./Core/Src/stack.o \
./Core/Src/stm32f7xx_hal_msp.o \
./Core/Src/stm32f7xx_hal_timebase_tim.o \
./Core/Src/stm32f7xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f7xx.o \
./Core/Src/timer.o \
./Core/Src/ui.o 

C_DEPS += \
./Core/Src/bus.d \
./Core/Src/cart.d \
./Core/Src/cpu.d \
./Core/Src/cpu_fetch.d \
./Core/Src/cpu_proc.d \
./Core/Src/cpu_util.d \
./Core/Src/dbg.d \
./Core/Src/dma.d \
./Core/Src/emu.d \
./Core/Src/freertos.d \
./Core/Src/gamepad.d \
./Core/Src/instructions.d \
./Core/Src/interrupts.d \
./Core/Src/io.d \
./Core/Src/lcd.d \
./Core/Src/main.d \
./Core/Src/ppu.d \
./Core/Src/ppu_pipeline.d \
./Core/Src/ppu_sm.d \
./Core/Src/ram.d \
./Core/Src/stack.d \
./Core/Src/stm32f7xx_hal_msp.d \
./Core/Src/stm32f7xx_hal_timebase_tim.d \
./Core/Src/stm32f7xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f7xx.d \
./Core/Src/timer.d \
./Core/Src/ui.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F769xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/bus.d ./Core/Src/bus.o ./Core/Src/cart.d ./Core/Src/cart.o ./Core/Src/cpu.d ./Core/Src/cpu.o ./Core/Src/cpu_fetch.d ./Core/Src/cpu_fetch.o ./Core/Src/cpu_proc.d ./Core/Src/cpu_proc.o ./Core/Src/cpu_util.d ./Core/Src/cpu_util.o ./Core/Src/dbg.d ./Core/Src/dbg.o ./Core/Src/dma.d ./Core/Src/dma.o ./Core/Src/emu.d ./Core/Src/emu.o ./Core/Src/freertos.d ./Core/Src/freertos.o ./Core/Src/gamepad.d ./Core/Src/gamepad.o ./Core/Src/instructions.d ./Core/Src/instructions.o ./Core/Src/interrupts.d ./Core/Src/interrupts.o ./Core/Src/io.d ./Core/Src/io.o ./Core/Src/lcd.d ./Core/Src/lcd.o ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/ppu.d ./Core/Src/ppu.o ./Core/Src/ppu_pipeline.d ./Core/Src/ppu_pipeline.o ./Core/Src/ppu_sm.d ./Core/Src/ppu_sm.o ./Core/Src/ram.d ./Core/Src/ram.o ./Core/Src/stack.d ./Core/Src/stack.o ./Core/Src/stm32f7xx_hal_msp.d ./Core/Src/stm32f7xx_hal_msp.o ./Core/Src/stm32f7xx_hal_timebase_tim.d ./Core/Src/stm32f7xx_hal_timebase_tim.o ./Core/Src/stm32f7xx_it.d ./Core/Src/stm32f7xx_it.o ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/system_stm32f7xx.d ./Core/Src/system_stm32f7xx.o ./Core/Src/timer.d ./Core/Src/timer.o ./Core/Src/ui.d ./Core/Src/ui.o

.PHONY: clean-Core-2f-Src

