-include target/stm32_common/Makefile.mk
SOURCE_DIRS+=target/stm32c0
INCLUDE_PATH+=target/stm32c0
INCLUDE_PATH+=lib/CMSIS/Device/ST/STM32C0xx/Include
OPTS+=-mcpu=cortex-m0plus -march=armv6-m -mthumb
C_MACROS+=STM32C0XX STM32_HASDMAMUX SAMPLE_TABLE_SIZE=64
GLOBAL_DEPS+=target/stm32c0/Makefile.mk

MODULES=oscilloscope pwm_generator voltmeter pwm pwm_input

PERIPHERALS_CPP+=adc
PERIPHERALS+=uart

