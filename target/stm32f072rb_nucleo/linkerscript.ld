MEMORY
{
  ram (rwx) : ORIGIN = 0x20000000, LENGTH = 16K
  rom (rx)  : ORIGIN = 0x08000000, LENGTH = 128K
}

STACK_SIZE = 0x200;
HEAP_SIZE = 0x1000;

INCLUDE target/stm32_common/linkerscript.ld
