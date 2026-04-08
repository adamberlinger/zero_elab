MEMORY
{
  ram (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
  rom (rx)  : ORIGIN = 0x08000000, LENGTH = 512K
}

STACK_SIZE = 0x400;
HEAP_SIZE = 120K;

INCLUDE target/stm32_common/linkerscript.ld
