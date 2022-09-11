-include target/stm32l0/Makefile.mk
SOURCE_DIRS+=target/stm32l072kz
INCLUDE_PATH+=target/stm32l072kz
C_MACROS+=STM32L072xx SAMPLE_TABLE_SIZE=256
GLOBAL_DEPS+=target/stm32l072kz/Makefile.mk
IS_ABSTRACT=no
PERIPHERALS_CPP+=dac
IS_DFU=yes

-include target/stm32x0_usb_cdc/Makefile.mk

#Specific variables for Keil project generation
MDK_CPU=IROM(0x8000000,0x8000) IRAM(0x20000000,0x1800) CPUTYPE(\"Cortex-M0\") CLOCK(48000000) ELITTLE
MDK_SVD=$$$$Device:STM32F042F6$$SVD\STM32F042x.svd
KEIL_SCT=target/stm32f042f6/keil.sct
