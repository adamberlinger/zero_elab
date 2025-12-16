-include target/stm32g0/Makefile.mk
SOURCE_DIRS+=target/stm32g051f8
INCLUDE_PATH+=target/stm32g051f8
# Make compiler think we use G051
C_MACROS+=STM32G051xx
GLOBAL_DEPS+=target/stm32g051f8/Makefile.mk
IS_ABSTRACT=no

PERIPHERALS_CPP+=dac
#MODULES+=generator

