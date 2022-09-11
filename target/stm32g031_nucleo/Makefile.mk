-include target/stm32g0/Makefile.mk
SOURCE_DIRS+=target/stm32g031_nucleo
INCLUDE_PATH+=target/stm32g031_nucleo
C_MACROS+=STM32G031xx
GLOBAL_DEPS+=target/stm32g031_nucleo/Makefile.mk
IS_ABSTRACT=no
