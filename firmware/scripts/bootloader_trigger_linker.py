Import("env")

import os
import re
import subprocess
import sys
from pathlib import Path


def _parse_int(value):
    if isinstance(value, int):
        return value
    return int(str(value), 0)


def _align_up(value, alignment):
    return (value + alignment - 1) // alignment * alignment


def _action_path_env():
    action_env = os.environ.copy()
    action_env.update(env.get("ENV", {}))
    return action_env


def _tool_path(name):
    tool = env.subst(f"${name.upper()}")
    return tool if tool and not tool.startswith("$") else name


def _run_command(args, description):
    result = subprocess.run(
        args,
        cwd=env.subst("$PROJECT_DIR"),
        env=_action_path_env(),
        encoding="utf-8",
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    if result.returncode != 0:
        if result.stdout:
            print(result.stdout, end="")
        if result.stderr:
            print(result.stderr, end="", file=sys.stderr)
        raise RuntimeError(f"{description} failed with exit code {result.returncode}")
    return result.stdout


def _stack_usage_script_path():
    return Path(env.subst("$PROJECT_DIR")).parent / "stack_usage.py"


def _read_symbol_address(elf_path, symbol_names):
    output = _run_command(
        [_tool_path("nm"), "--defined-only", str(elf_path)],
        "reading ELF symbols",
    )
    symbols = {}
    for line in output.splitlines():
        parts = line.split()
        if len(parts) >= 3 and parts[-1] in symbol_names:
            symbols[parts[-1]] = int(parts[0], 16)
    for symbol_name in symbol_names:
        if symbol_name in symbols:
            return symbols[symbol_name]
    raise RuntimeError(
        "could not find any of these symbols in the ELF: "
        + ", ".join(symbol_names)
    )


def _stack_usage_results(elf_path, su_root):
    stack_usage_script = _stack_usage_script_path()
    if not stack_usage_script.exists():
        raise RuntimeError(f"missing stack usage analyzer: {stack_usage_script}")

    output = _run_command(
        [sys.executable, str(stack_usage_script), str(elf_path), str(su_root)],
        "stack usage analysis",
    )
    results = []
    for line in output.splitlines():
        match = re.match(r"\s*(\d+):\s+(\S+)\s+\((.*)\)", line)
        if match:
            results.append((int(match.group(1)), match.group(2), match.group(3)))
    if not results:
        raise RuntimeError("stack usage analysis produced no stack-size results")
    return results


def _report_maximum_stack_address(target, source, env):
    del source

    elf_path = None
    for node in target:
        path = Path(str(node))
        if path.suffix == ".elf":
            elf_path = path
            break
    if elf_path is None:
        elf_path = Path(env.subst("$BUILD_DIR")) / f"{env.subst('$PROGNAME')}.elf"

    try:
        results = _stack_usage_results(elf_path, Path(env.subst("$BUILD_DIR")))
        max_stack_usage, entry_function, stack_path = max(
            results, key=lambda result: result[0]
        )
        ram_data_end = _read_symbol_address(elf_path, ["end", "_end", "_ebss"])
        max_stack_address = _align_up(ram_data_end + max_stack_usage, 8)
        ram_end = ram_origin + ram_size
        headroom = ram_end - max_stack_address

        print(
            "Maximum stack address: "
            f"0x{max_stack_address:08x} "
            f"({max_stack_usage} bytes via {entry_function})"
        )
        print(
            "RAM data end: "
            f"0x{ram_data_end:08x}; "
            f"RAM end: 0x{ram_end:08x}; "
            f"stack headroom: {headroom} bytes"
        )
        print(f"Maximum stack path: {stack_path}")

        if max_stack_address > ram_end:
            print(
                "Error: maximum stack address exceeds available RAM",
                file=sys.stderr,
            )
            return 1
    except RuntimeError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    return 0


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

env.Append(CCFLAGS=["-fstack-usage"])

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
env.AddPostAction(
    "checkprogsize",
    env.VerboseAction(_report_maximum_stack_address, "Analyzing stack usage"),
)
