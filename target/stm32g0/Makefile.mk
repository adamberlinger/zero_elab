-include target/stm32_common/Makefile.mk
SOURCE_DIRS+=target/stm32g0
INCLUDE_PATH+=target/stm32g0
INCLUDE_PATH+=lib/CMSIS/Device/ST/STM32G0xx/Include
OPTS+=-mcpu=cortex-m0plus -march=armv6-m -mthumb
C_MACROS+=STM32G0XX STM32_HASDMAMUX
GLOBAL_DEPS+=target/stm32g0/Makefile.mk

MODULES=oscilloscope generator voltmeter pwm pwm_input

PERIPHERALS_CPP+=adc
PERIPHERALS+=uart

