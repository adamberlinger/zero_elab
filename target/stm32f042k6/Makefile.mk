-include target/stm32f0/Makefile.mk
SOURCE_DIRS+=target/stm32f042k6
INCLUDE_PATH+=target/stm32f042k6
C_MACROS+=STM32F042x6
GLOBAL_DEPS+=target/stm32f042k6/Makefile.mk
IS_ABSTRACT=no
#generate DFU file into ZIP package
IS_DFU=yes

#-include target/stm32f0_usb_vcp/Makefile.mk
-include target/stm32x0_usb_cdc/Makefile.mk

#Specific variables for Keil project generation
MDK_CPU=IROM(0x8000000,0x8000) IRAM(0x20000000,0x1800) CPUTYPE(\"Cortex-M0\") CLOCK(48000000) ELITTLE
MDK_SVD=$$$$Device:STM32F042K6$$SVD\STM32F042x.svd
KEIL_SCT=target/stm32f042k6/keil.sct
