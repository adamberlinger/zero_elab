-include target/stm32c0/Makefile.mk
SOURCE_DIRS+=target/stm32c031c6_nucleo
INCLUDE_PATH+=target/stm32c031c6_nucleo
C_MACROS+=STM32C031xx
GLOBAL_DEPS+=target/stm32c031c6_nucleo/Makefile.mk
IS_ABSTRACT=no
