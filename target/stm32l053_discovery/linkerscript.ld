MEMORY
{
  ram (rwx) : ORIGIN = 0x20000000, LENGTH = 8K
  rom (rx)  : ORIGIN = 0x08000000, LENGTH = 64K
}

STACK_SIZE = 0x200;
HEAP_SIZE = 4096;

INCLUDE target/stm32_common/linkerscript.ld
