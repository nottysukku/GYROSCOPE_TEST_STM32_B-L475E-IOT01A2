################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32L4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := GYROSCOPE_TEST_IN_OFFICE
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
GYROSCOPE_TEST_IN_OFFICE.elf \

MAP_FILES += \
GYROSCOPE_TEST_IN_OFFICE.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
GYROSCOPE_TEST_IN_OFFICE.list \


# All Target
all: main-build

# Main-build Target
main-build: GYROSCOPE_TEST_IN_OFFICE.elf secondary-outputs

# Tool invocations
GYROSCOPE_TEST_IN_OFFICE.elf GYROSCOPE_TEST_IN_OFFICE.map: $(OBJS) $(USER_OBJS) C:\Users\sukri\STM32CubeIDE\workspace_1.18.2\GYROSCOPE_TEST_IN_OFFICE\STM32L475VGTX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "GYROSCOPE_TEST_IN_OFFICE.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\sukri\STM32CubeIDE\workspace_1.18.2\GYROSCOPE_TEST_IN_OFFICE\STM32L475VGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="GYROSCOPE_TEST_IN_OFFICE.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

GYROSCOPE_TEST_IN_OFFICE.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "GYROSCOPE_TEST_IN_OFFICE.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) GYROSCOPE_TEST_IN_OFFICE.elf GYROSCOPE_TEST_IN_OFFICE.list GYROSCOPE_TEST_IN_OFFICE.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
