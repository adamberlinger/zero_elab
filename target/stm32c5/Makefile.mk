-include target/stm32_common/Makefile.mk
SOURCE_DIRS+=target/stm32c5
INCLUDE_PATH+=target/stm32c5
INCLUDE_PATH+=lib/CMSIS/Device/ST/STM32C5xx/Include
OPTS+=-mcpu=cortex-m33 -mthumb
C_MACROS+=STM32C5XX SAMPLE_TABLE_SIZE=64 STM32_HASGPDMA
GLOBAL_DEPS+=target/stm32c5/Makefile.mk

MODULES=pwm generator

PERIPHERALS_CPP+=dac
#PERIPHERALS_CPP+=adc
#PERIPHERALS+=uart

