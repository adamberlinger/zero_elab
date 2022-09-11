-include target/stm32g0/Makefile.mk
SOURCE_DIRS+=target/stm32g030j6
INCLUDE_PATH+=target/stm32g030j6
# Make compiler think we use G031
C_MACROS+=STM32G031xx STM32G0_SWITCH_PA
GLOBAL_DEPS+=target/stm32g030j6/Makefile.mk
IS_ABSTRACT=no
