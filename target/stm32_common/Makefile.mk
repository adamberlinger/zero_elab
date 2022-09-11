-include target/arm_common/Makefile.mk
GLOBAL_DEPS+=target/stm32_common/Makefile.mk

INCLUDE_PATH+=target/stm32_common
SOURCE_DIRS+=target/stm32_common
MODULES+=pwm_input pwm pulse_counter
PERIPHERALS+=timer
