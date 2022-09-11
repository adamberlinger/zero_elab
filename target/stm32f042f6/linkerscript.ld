MEMORY
{
  ram (rwx) : ORIGIN = 0x20000000, LENGTH = 6K
  rom (rx)  : ORIGIN = 0x08000000, LENGTH = 32K
}

STACK_SIZE = 0x200;
/*HEAP_SIZE = 3700;*/
HEAP_SIZE = (4*1024+192);

INCLUDE target/stm32_common/linkerscript.ld
