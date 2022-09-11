-include target/stm32_common/Makefile.mk
SOURCE_DIRS+=target/stm32f3
INCLUDE_PATH+=target/stm32f3
INCLUDE_PATH+=lib/CMSIS/Device/ST/STM32F3xx/Include
OPTS+=-mcpu=cortex-m4 -march=armv7e-m -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
C_MACROS+=STM32F3XX
GLOBAL_DEPS+=target/stm32f3/Makefile.mk

PERIPHERALS+=uart
PERIPHERALS_CPP+=adc dac
MODULES+=oscilloscope generator voltmeter
