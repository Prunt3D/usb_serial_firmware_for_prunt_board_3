Import("env")

from pathlib import Path


def _parse_int(value):
    if isinstance(value, int):
        return value
    return int(str(value), 0)


board = env.BoardConfig()
mcu = board.get("build.mcu", "").lower()
flash_origin = _parse_int(board.get("upload.offset_address", "0x08000000"))
flash_size = _parse_int(board.get("upload.maximum_size", 0))
ram_origin = 0x20000000
ram_size = _parse_int(board.get("upload.maximum_ram_size", 0))

# The STM32F04xxx/STM32F070x6 empty-check boot path samples the erased word at
# the start of flash. Keep that whole erase page free of normal application code.
boot_trigger_page_size = 0
if mcu.startswith("stm32f0"):
    boot_trigger_page_size = 2048 if flash_size > 65536 else 1024

build_dir = Path(env.subst("$BUILD_DIR"))
build_dir.mkdir(parents=True, exist_ok=True)
ldscript_path = build_dir / "bootloader_trigger.ld"

if boot_trigger_page_size:
    text_sections = """ .text : {
  KEEP(*(.vectors))
  . = __bootloader_trigger_page_size;
  *(.text*)
  . = ALIGN(4);
  *(.rodata*)
  . = ALIGN(4);
 } >rom :text =0x00bf"""
else:
    text_sections = """ .text : {
  KEEP(*(.vectors))
  *(.text*)
  . = ALIGN(4);
  *(.rodata*)
  . = ALIGN(4);
 } >rom :text"""

ldscript_path.write_text(
    f"""EXTERN(vector_table)
ENTRY(reset_handler)
MEMORY
{{
 ram (rwx) : ORIGIN = 0x{ram_origin:08x}, LENGTH = {ram_size}
 rom (rx) : ORIGIN = 0x{flash_origin:08x}, LENGTH = {flash_size}
}}

__bootloader_trigger_page_start = ORIGIN(rom);
__bootloader_trigger_page_size = {boot_trigger_page_size};
__bootloader_trigger_page_end = ORIGIN(rom) + __bootloader_trigger_page_size;

PHDRS
{{
 text PT_LOAD FLAGS(5);
 data PT_LOAD FLAGS(6);
}}

SECTIONS
{{
{text_sections}
 .preinit_array : {{
  . = ALIGN(4);
  __preinit_array_start = .;
  KEEP (*(.preinit_array))
  __preinit_array_end = .;
 }} >rom :text
 .init_array : {{
  . = ALIGN(4);
  __init_array_start = .;
  KEEP (*(SORT(.init_array.*)))
  KEEP (*(.init_array))
  __init_array_end = .;
 }} >rom :text
 .fini_array : {{
  . = ALIGN(4);
  __fini_array_start = .;
  KEEP (*(.fini_array))
  KEEP (*(SORT(.fini_array.*)))
  __fini_array_end = .;
 }} >rom :text
 .ARM.extab : {{
  *(.ARM.extab*)
 }} >rom :text
 .ARM.exidx : {{
  __exidx_start = .;
  *(.ARM.exidx*)
  __exidx_end = .;
 }} >rom :text
 . = ALIGN(4);
 _etext = .;
 .noinit (NOLOAD) : {{
  *(.noinit*)
 }} >ram :data
 . = ALIGN(4);
 .data : {{
  _data = .;
  *(.data*)
  *(.ramtext*)
  . = ALIGN(4);
  _edata = .;
 }} >ram AT >rom :data
 _data_loadaddr = LOADADDR(.data);
 .bss : {{
  *(.bss*)
  *(COMMON)
  . = ALIGN(4);
  _ebss = .;
 }} >ram :data
 /DISCARD/ : {{ *(.eh_frame) }}
 . = ALIGN(4);
 end = .;
}}
PROVIDE(_stack = ORIGIN(ram) + LENGTH(ram));
""",
    encoding="ascii",
)

board.update("build.ldscript", str(ldscript_path))
env.Replace(LDSCRIPT_PATH=str(ldscript_path))
