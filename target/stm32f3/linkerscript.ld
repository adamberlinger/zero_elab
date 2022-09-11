MEMORY
{
  ram (rwx) : ORIGIN = 0x20000000, LENGTH = 64K
  stack(rwx): ORIGIN = 0x20004FFC, LENGTH = 0K

  rom (rx)  : ORIGIN = 0x08000000, LENGTH = 512K
}

INCLUDE target/stm32_common/linkerscript.ld
