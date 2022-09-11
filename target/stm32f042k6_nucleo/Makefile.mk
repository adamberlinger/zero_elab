-include target/stm32f0/Makefile.mk
SOURCE_DIRS+=target/stm32f042k6_nucleo
INCLUDE_PATH+=target/stm32f042k6_nucleo
C_MACROS+=STM32F042x6
GLOBAL_DEPS+=target/stm32f042k6_nucleo/Makefile.mk
IS_ABSTRACT=no

#Specific variables for Keil project generation
MDK_DEVICE=STM32F042K6
MDK_CPU=IRAM(0x20000000-0x200017FF) IROM(0x8000000-0x8007FFF) CLOCK(8000000) CPUTYPE(\"Cortex-M0+\")
KEIL_SCT=target/stm32f042k6_nucleo/keil.sct
KEIL_S=target/stm32f042k6_nucleo/keil.s
MDK_SVD=Device:STM32F042K6$$SVD\STM32F042x.svd
