################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../camera_ptzcontrol.c \
../cmdqu.c \
../cyfxtx.c \
../cyfxuvcdscr.c \
../sensor.c \
../uvc.c 

S_UPPER_SRCS += \
../cyfx_gcc_startup.S 

OBJS += \
./camera_ptzcontrol.o \
./cmdqu.o \
./cyfx_gcc_startup.o \
./cyfxtx.o \
./cyfxuvcdscr.o \
./sensor.o \
./uvc.o 

S_UPPER_DEPS += \
./cyfx_gcc_startup.d 

C_DEPS += \
./camera_ptzcontrol.d \
./cmdqu.d \
./cyfxtx.d \
./cyfxuvcdscr.d \
./sensor.d \
./uvc.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Sourcery Windows GCC C Compiler'
	arm-none-eabi-gcc -DTX_ENABLE_EVENT_TRACE -DDEBUG -DCYU3P_FX3=1 -D__CYU3P_TX__=1 -I"C:\Program Files\Cypress\EZ-USB FX3 SDK\1.3\\firmware\u3p_firmware\inc" -O3 -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=arm926ej-s -mthumb-interwork -g -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

%.o: ../%.S
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Sourcery Windows GCC Assembler'
	arm-none-eabi-gcc -x assembler-with-cpp -I"C:\Program Files\Cypress\EZ-USB FX3 SDK\1.3\\firmware\u3p_firmware\inc" -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=arm926ej-s -mthumb-interwork -g -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


