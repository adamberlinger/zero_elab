-include target/stm32g0/Makefile.mk
SOURCE_DIRS+=target/stm32g030_any
INCLUDE_PATH+=target/stm32g030_any
# Make compiler think we use G031
C_MACROS+=STM32G031xx
GLOBAL_DEPS+=target/stm32g030_any/Makefile.mk
IS_ABSTRACT=no
