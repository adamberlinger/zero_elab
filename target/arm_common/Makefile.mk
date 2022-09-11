GLOBAL_DEPS+=target/arm_common/Makefile.mk target/arm_common/linkerscript.ld
SOURCE_DIRS+=target/arm_common
INCLUDE_PATH+=lib/CMSIS/Include
INCLUDE_PATH+=target/arm_common
TOOL_PREFIX=arm-none-eabi-

KEIL_S=target/arm_common/keil.s
