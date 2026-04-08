-include target/stm32c5/Makefile.mk
SOURCE_DIRS+=target/stm32c562re_nucleo
INCLUDE_PATH+=target/stm32c562re_nucleo
C_MACROS+=STM32C562xx
GLOBAL_DEPS+=target/stm32c562re_nucleo/Makefile.mk
IS_ABSTRACT=no

-include target/stm32x0_usb_cdc/Makefile.mk

