-include target/stm32x0_common/Makefile.mk
SOURCE_DIRS+=target/stm32l0
INCLUDE_PATH+=target/stm32l0
INCLUDE_PATH+=lib/CMSIS/Device/ST/STM32L0xx/Include
OPTS+=-mcpu=cortex-m0plus -march=armv6-m -mthumb
C_MACROS+=STM32L0XX
GLOBAL_DEPS+=target/stm32l0/Makefile.mk

MODULES=oscilloscope generator voltmeter pwm pwm_input
