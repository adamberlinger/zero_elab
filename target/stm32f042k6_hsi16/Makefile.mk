-include target/stm32f042k6/Makefile.mk
GLOBAL_DEPS+=target/stm32f042k6_hsi16/Makefile.mk
C_MACROS+=STM32F0_USE_HSI
LINKER_SCRIPT=target/stm32f042k6/linkerscript.ld