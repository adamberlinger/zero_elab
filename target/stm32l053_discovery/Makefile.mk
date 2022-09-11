-include target/stm32l0/Makefile.mk
SOURCE_DIRS+=target/stm32l053_discovery
INCLUDE_PATH+=target/stm32l053_discovery
C_MACROS+=STM32L053xx SAMPLE_TABLE_SIZE=64
GLOBAL_DEPS+=target/stm32l053_discovery/Makefile.mk
IS_ABSTRACT=no
PERIPHERALS_CPP+=dac

-include target/stm32x0_usb_cdc/Makefile.mk

#Specific variables for Keil project generation
MDK_CPU=IROM(0x8000000,0x8000) IRAM(0x20000000,0x1800) CPUTYPE(\"Cortex-M0\") CLOCK(48000000) ELITTLE
MDK_SVD=$$$$Device:STM32F042F6$$SVD\STM32F042x.svd
KEIL_SCT=target/stm32f042f6/keil.sct
