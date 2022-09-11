MEMORY
{
  ram (rwx) : ORIGIN = 0x20000000, LENGTH = 20K
  rom (rx)  : ORIGIN = 0x08000000, LENGTH = 192K
}

STACK_SIZE = 0x400;
HEAP_SIZE = 17K;

INCLUDE target/stm32_common/linkerscript.ld
