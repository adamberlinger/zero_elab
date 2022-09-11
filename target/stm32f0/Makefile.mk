-include target/stm32x0_common/Makefile.mk
SOURCE_DIRS+=target/stm32f0
INCLUDE_PATH+=target/stm32f0
INCLUDE_PATH+=lib/CMSIS/Device/ST/STM32F0xx/Include
OPTS+=-mcpu=cortex-m0 -march=armv6-m -mthumb
C_MACROS+=STM32F0XX
GLOBAL_DEPS+=target/stm32f0/Makefile.mk

MODULES=oscilloscope generator voltmeter pwm pwm_input
