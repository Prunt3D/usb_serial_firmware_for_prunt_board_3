#include "bootloader.h"
#include <stdint.h>

static constexpr uint32_t FLASH_BASE_ADDR = 0x40022000;
static constexpr uint32_t FLASH_KEYR_ADDR = FLASH_BASE_ADDR + 0x04;
static constexpr uint32_t FLASH_SR_ADDR = FLASH_BASE_ADDR + 0x0c;
static constexpr uint32_t FLASH_CR_ADDR = FLASH_BASE_ADDR + 0x10;
static constexpr uint32_t FLASH_AR_ADDR = FLASH_BASE_ADDR + 0x14;

static constexpr uint32_t FLASH_KEY1 = 0x45670123;
static constexpr uint32_t FLASH_KEY2 = 0xcdef89ab;
static constexpr uint32_t FLASH_SR_EOP = 1u << 5;
static constexpr uint32_t FLASH_SR_WRPRTERR = 1u << 4;
static constexpr uint32_t FLASH_SR_PGERR = 1u << 2;
static constexpr uint32_t FLASH_SR_BSY = 1u << 0;
static constexpr uint32_t FLASH_CR_LOCK = 1u << 7;
static constexpr uint32_t FLASH_CR_STRT = 1u << 6;
static constexpr uint32_t FLASH_CR_PER = 1u << 1;

static constexpr uint32_t USER_FLASH_BASE_ADDR = 0x08000000;
static constexpr uint32_t SYSTEM_MEMORY_BASE_ADDR = 0x1fffc400;

static constexpr uint32_t RCC_APB2ENR_ADDR = 0x40021018;
static constexpr uint32_t RCC_APB2ENR_SYSCFGCOMPEN = 1u << 0;
static constexpr uint32_t SYSCFG_CFGR1_ADDR = 0x40010000;
static constexpr uint32_t SYSCFG_CFGR1_MEM_MODE = 3u << 0;
static constexpr uint32_t SYSCFG_CFGR1_MEM_MODE_SYSTEM = 1u << 0;
static constexpr uint32_t STK_CSR_ADDR = 0xe000e010;
static constexpr uint32_t NVIC_ICER_ADDR = 0xe000e180;
static constexpr uint32_t NVIC_ICPR_ADDR = 0xe000e280;
static constexpr uint32_t SCB_ICSR_ADDR = 0xe000ed04;
static constexpr uint32_t SCB_ICSR_PENDSVCLR = 1u << 27;
static constexpr uint32_t SCB_ICSR_PENDSTCLR = 1u << 25;

static inline __attribute__((always_inline)) volatile uint32_t &reg32(uint32_t address)
{
    return *reinterpret_cast<volatile uint32_t *>(address);
}

static inline __attribute__((always_inline)) void wait_for_flash()
{
    while ((reg32(FLASH_SR_ADDR) & FLASH_SR_BSY) != 0) {
    }
}

static void __attribute__((section(".ramtext"), noinline, long_call, noreturn)) erase_boot_trigger_page_and_jump()
{
    __asm volatile("cpsid i" ::: "memory");

    wait_for_flash();
    reg32(FLASH_SR_ADDR) = FLASH_SR_EOP | FLASH_SR_WRPRTERR | FLASH_SR_PGERR;

    if ((reg32(FLASH_CR_ADDR) & FLASH_CR_LOCK) != 0) {
        reg32(FLASH_KEYR_ADDR) = FLASH_KEY1;
        reg32(FLASH_KEYR_ADDR) = FLASH_KEY2;
    }

    reg32(FLASH_CR_ADDR) |= FLASH_CR_PER;
    reg32(FLASH_AR_ADDR) = USER_FLASH_BASE_ADDR;
    reg32(FLASH_CR_ADDR) |= FLASH_CR_STRT;
    wait_for_flash();
    reg32(FLASH_CR_ADDR) &= ~FLASH_CR_PER;
    reg32(FLASH_SR_ADDR) = FLASH_SR_EOP | FLASH_SR_WRPRTERR | FLASH_SR_PGERR;

    reg32(STK_CSR_ADDR) = 0;
    reg32(SCB_ICSR_ADDR) = SCB_ICSR_PENDSVCLR | SCB_ICSR_PENDSTCLR;
    reg32(NVIC_ICER_ADDR) = 0xffffffff;
    reg32(NVIC_ICPR_ADDR) = 0xffffffff;

    reg32(RCC_APB2ENR_ADDR) |= RCC_APB2ENR_SYSCFGCOMPEN;
    reg32(SYSCFG_CFGR1_ADDR) =
        (reg32(SYSCFG_CFGR1_ADDR) & ~SYSCFG_CFGR1_MEM_MODE) | SYSCFG_CFGR1_MEM_MODE_SYSTEM;

    const uint32_t boot_stack = reg32(SYSTEM_MEMORY_BASE_ADDR);
    const uint32_t boot_reset = reg32(SYSTEM_MEMORY_BASE_ADDR + 4);

    __asm volatile("dsb\nisb" ::: "memory");
    __asm volatile(
        "movs r0, #0\n"
        "msr control, r0\n"
        "msr msp, %[stack]\n"
        "cpsie i\n"
        "isb\n"
        "bx %[reset]\n"
        :
        : [stack] "r"(boot_stack), [reset] "r"(boot_reset)
        : "r0", "memory");

    while (1) {
    }
}

void bootloader_enter_system_memory_boot_mode()
{
    erase_boot_trigger_page_and_jump();
}
