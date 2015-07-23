################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := cs-rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include .metadata/.plugins/org.eclipse.cdt.make.core/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_LIST += \
cyfxuvc.lst \

SECONDARY_SIZE += \
cyfxuvc.siz \


# All Target
all: cyfxuvc.elf secondary-outputs

# Tool invocations
cyfxuvc.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: ARM Sourcery Windows GCC C Linker'
	arm-none-eabi-gcc $(OBJS) $(USER_OBJS) $(LIBS) -T"C:\Program Files\Cypress\EZ-USB FX3 SDK\1.3\\firmware\common\fx3.ld" -nostartfiles -Wl,-Map,cyfxuvc.map -Wl,-d -Wl,--no-wchar-size-warning -Wl,--gc-sections -Wl,--entry,CyU3PFirmwareEntry "C:\Program Files\Cypress\EZ-USB FX3 SDK\1.3\\firmware\u3p_firmware\lib\fx3_debug\cyfxapi.a" "C:\Program Files\Cypress\EZ-USB FX3 SDK\1.3\\firmware\u3p_firmware\lib\fx3_debug\cyu3threadx.a" "C:\Program Files\Cypress\EZ-USB FX3 SDK\1.3\\firmware\u3p_firmware\lib\fx3_debug\cyu3lpp.a"  "C:\Program Files\Cypress\EZ-USB FX3 SDK\1.3\\arm-none-eabi\lib\libc.a" "C:\Program Files\Cypress\EZ-USB FX3 SDK\1.3\\lib\gcc\arm-none-eabi\4.5.2\libgcc.a" -mcpu=arm926ej-s -mthumb-interwork -g -gdwarf-2 -o "cyfxuvc.elf"
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

cyfxuvc.lst: cyfxuvc.elf
	@echo 'Invoking: ARM Sourcery Windows GNU Create Listing'
	arm-none-eabi-objdump -h -S cyfxuvc.elf > "cyfxuvc.lst"
	@echo 'Finished building: $@'
	@echo ' '

cyfxuvc.siz: cyfxuvc.elf
	@echo 'Invoking: ARM Sourcery Windows GNU Print Size'
	arm-none-eabi-size  --format=berkeley cyfxuvc.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(OBJS)$(SECONDARY_LIST)$(SECONDARY_SIZE)$(ASM_DEPS)$(S_UPPER_DEPS)$(C_DEPS) cyfxuvc.elf
	-@echo ' '

post-build:
	-C:\Program Files\Cypress\EZ-USB FX3 SDK\1.3\\util\elf2img\elf2img.exe -i cyfxuvc.elf -o cyfxuvc.img
	-@echo ' '

secondary-outputs: $(SECONDARY_LIST) $(SECONDARY_SIZE)

.PHONY: all clean dependents
.SECONDARY: post-build

-include ../makefile.targets
