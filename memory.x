MEMORY
{
  /* Leave 8k for the default bootloader on the Circuit Playground Express */
  FLASH (rx) : ORIGIN = 0x00000000 + 8K, LENGTH = 256K - 8K
  RAM (xrw)  : ORIGIN = 0x20000000, LENGTH = 31K + 768
  PANDUMP: ORIGIN = 0x20007F00, LENGTH = 256
}
_stack_start = ORIGIN(RAM) + LENGTH(RAM);

_panic_dump_start = ORIGIN(PANDUMP);
_panic_dump_end   = ORIGIN(PANDUMP) + LENGTH(PANDUMP);
