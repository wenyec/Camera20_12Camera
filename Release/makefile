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
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_FLASH += \
USB32HDV10Snap.hex \

SECONDARY_LIST += \
USB32HDV10Snap.lst \

SECONDARY_SIZE += \
USB32HDV10Snap.siz \


# All Target
all: USB32HDV10Snap.elf secondary-outputs

# Tool invocations
USB32HDV10Snap.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: ARM Sourcery Windows GCC C Linker'
	arm-none-eabi-gcc -nostartfiles -Wl,-Map,USB32HDV10Snap.map -mcpu=cortex-m4 -mthumb -o "USB32HDV10Snap.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

USB32HDV10Snap.hex: USB32HDV10Snap.elf
	@echo 'Invoking: ARM Sourcery Windows GNU Create Flash Image'
	arm-none-eabi-objcopy -O ihex USB32HDV10Snap.elf  "USB32HDV10Snap.hex"
	@echo 'Finished building: $@'
	@echo ' '

USB32HDV10Snap.lst: USB32HDV10Snap.elf
	@echo 'Invoking: ARM Sourcery Windows GNU Create Listing'
	arm-none-eabi-objdump -h -S USB32HDV10Snap.elf > "USB32HDV10Snap.lst"
	@echo 'Finished building: $@'
	@echo ' '

USB32HDV10Snap.siz: USB32HDV10Snap.elf
	@echo 'Invoking: ARM Sourcery Windows GNU Print Size'
	arm-none-eabi-size  --format=berkeley USB32HDV10Snap.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(SECONDARY_SIZE)$(OBJS)$(C_DEPS)$(ASM_DEPS)$(SECONDARY_FLASH)$(EXECUTABLES)$(SECONDARY_LIST)$(S_UPPER_DEPS) USB32HDV10Snap.elf
	-@echo ' '

secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_LIST) $(SECONDARY_SIZE)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
