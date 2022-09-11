-include target/stm32f3/Makefile.mk
SOURCE_DIRS+=target/stm32f303ze_nucleo
INCLUDE_PATH+=target/stm32f303ze_nucleo
C_MACROS+=STM32F303xE TARGET_USE_HSE TARGET_HSE_BYP STM_NUCLEO144
GLOBAL_DEPS+=target/stm32f303ze_nucleo/Makefile.mk
IS_ABSTRACT=no

#Specific variables for Keil project generation
MDK_CPU=IRAM(0x20000000-0x20003FFF) IROM(0x8000000-0x800FFFF) CLOCK(8000000) CPUTYPE(\"Cortex-M4\") FPU2
MDK_SVD=$$$$Device:STM32F303ZE$$SVD\STM32F303x.svd
