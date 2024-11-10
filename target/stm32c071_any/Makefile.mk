-include target/stm32c0/Makefile.mk
SOURCE_DIRS+=target/stm32c071_any
INCLUDE_PATH+=targetstm32c071_any
C_MACROS+=STM32C071xx
GLOBAL_DEPS+=target/stm32c071_any/Makefile.mk
IS_ABSTRACT=no

-include target/stm32x0_usb_cdc/Makefile.mk
