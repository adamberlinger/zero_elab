-include target/stm32_common/Makefile.mk
SOURCE_DIRS+=target/stm32f4
INCLUDE_PATH+=target/stm32f4
INCLUDE_PATH+=lib/CMSIS/Device/ST/STM32F4xx/Include
OPTS+=-mcpu=cortex-m4 -march=armv7e-m -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
C_MACROS+=STM32F4XX
GLOBAL_DEPS+=target/stm32f4/Makefile.mk

#Specific variables for Keil project generation
MDK_CPU=IRAM(0x20000000-0x20003FFF) IROM(0x8000000-0x800FFFF) CLOCK(8000000) CPUTYPE(\"Cortex-M4\") FPU2
MDK_SVD=$$$$Device:STM32F303K8$$SVD\STM32F303x.svd
