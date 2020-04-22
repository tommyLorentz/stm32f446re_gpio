################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include drivers/src/subdir.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
132.stm32f446re_spi.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
132.stm32f446re_spi.list \


# All Target
all: 132.stm32f446re_spi.elf secondary-outputs

# Tool invocations
132.stm32f446re_spi.elf: $(OBJS) $(USER_OBJS) C:\workspace\StmCubeIde\132.stm32f446re_spi\STM32F446RETX_FLASH.ld
	arm-none-eabi-gcc -o "132.stm32f446re_spi.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\workspace\StmCubeIde\132.stm32f446re_spi\STM32F446RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="132.stm32f446re_spi.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

132.stm32f446re_spi.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "132.stm32f446re_spi.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
