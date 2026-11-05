#include "bootloader.h"
#include <libopencm3/stm32/flash.h>
#include <stdint.h>

static constexpr uint32_t USER_FLASH_BASE_ADDR = 0x08000000;

// Inlined rather than calling libopencm3 because the erase below must not call into flash.
static inline __attribute__((always_inline)) void wait_for_flash()
{
    while ((FLASH_SR & FLASH_SR_BSY) != 0) {
    }
}

// The F042 only updates its flash-empty flag when option bytes load, so SYSRESETREQ would boot
// the erased page instead of the system bootloader. OBL_LAUNCH reloads the option bytes and
// resets the MCU, so the bootloader starts from real reset state. If the erase fails, the flag
// stays clear and the application simply restarts.
static void __attribute__((section(".ramtext"), noinline, long_call, noreturn)) erase_boot_trigger_page_and_reload_option_bytes()
{
    __asm volatile("cpsid i" ::: "memory");

    wait_for_flash();
    FLASH_SR = FLASH_SR_EOP | FLASH_SR_WRPRTERR | FLASH_SR_PGERR;

    if ((FLASH_CR & FLASH_CR_LOCK) != 0) {
        FLASH_KEYR = FLASH_KEYR_KEY1;
        FLASH_KEYR = FLASH_KEYR_KEY2;
    }

    FLASH_CR |= FLASH_CR_PER;
    FLASH_AR = USER_FLASH_BASE_ADDR;
    FLASH_CR |= FLASH_CR_STRT;
    wait_for_flash();
    FLASH_CR &= ~FLASH_CR_PER;
    FLASH_SR = FLASH_SR_EOP | FLASH_SR_WRPRTERR | FLASH_SR_PGERR;

    FLASH_CR |= FLASH_CR_OBL_LAUNCH;

    while (1) {
    }
}

void bootloader_enter_system_memory_boot_mode()
{
    erase_boot_trigger_page_and_reload_option_bytes();
}
