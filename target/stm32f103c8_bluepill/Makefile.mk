-include target/stm32f1/Makefile.mk
SOURCE_DIRS+=target/stm32f103c8_bluepill
INCLUDE_PATH+=target/stm32f103c8_bluepill
C_MACROS+=STM32F103xB TARGET_USE_HSE
GLOBAL_DEPS+=target/stm32f103c8_bluepill/Makefile.mk
IS_ABSTRACT=no
#generate DFU file into ZIP package
IS_DFU=yes

-include target/stm32x0_usb_cdc/Makefile.mk
