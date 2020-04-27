################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include drivers/src/subdir.mk
-include drivers/inc/subdir.mk
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
179.stm43f446re_i2c.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
179.stm43f446re_i2c.list \

OBJCOPY_BIN += \
179.stm43f446re_i2c.bin \


# All Target
all: 179.stm43f446re_i2c.elf secondary-outputs

# Tool invocations
179.stm43f446re_i2c.elf: $(OBJS) $(USER_OBJS) C:\workspace\StmCubeIde\179.stm43f446re_i2c\STM32F446RETX_FLASH.ld
	arm-none-eabi-gcc -o "179.stm43f446re_i2c.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\workspace\StmCubeIde\179.stm43f446re_i2c\STM32F446RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="179.stm43f446re_i2c.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

179.stm43f446re_i2c.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "179.stm43f446re_i2c.list"
	@echo 'Finished building: $@'
	@echo ' '

179.stm43f446re_i2c.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "179.stm43f446re_i2c.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
