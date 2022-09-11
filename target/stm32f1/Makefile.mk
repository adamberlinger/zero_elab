-include target/stm32_common/Makefile.mk
SOURCE_DIRS+=target/stm32f1
INCLUDE_PATH+=target/stm32f1
INCLUDE_PATH+=lib/CMSIS/Device/ST/STM32F1xx/Include
OPTS+=-mcpu=cortex-m3 -march=armv7-m -mthumb
C_MACROS+=STM32F1XX
GLOBAL_DEPS+=target/stm32f1/Makefile.mk

PERIPHERALS_CPP+=adc
MODULES=voltmeter pwm pwm_input oscilloscope
