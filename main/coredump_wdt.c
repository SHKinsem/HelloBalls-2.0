/*
 * Lets the core dump survive an interrupt watchdog panic.
 *
 * The panic handler disables the watchdogs before writing the core dump but
 * leaves the interrupt watchdog's interrupt pending. The dump lowers the CPU
 * interrupt level for a moment (window spill), the pending level-4 interrupt
 * fires again, the handler reports "Re-entered core dump" and reboots with
 * the coredump partition still erased. Clear the pending watchdog interrupts
 * before the dump starts. Wired in with -Wl,--wrap=esp_core_dump_write.
 */
#include "hal/wdt_hal.h"
#include "soc/timer_group_struct.h"

void __real_esp_core_dump_write(void *info);

static void clear_pending(timg_dev_t *dev, wdt_inst_t inst)
{
    wdt_hal_context_t ctx = {.inst = inst, .mwdt_dev = dev};
    wdt_hal_write_protect_disable(&ctx);
    wdt_hal_handle_intr(&ctx);  // feed and clear the interrupt status
    wdt_hal_write_protect_enable(&ctx);
}

void __wrap_esp_core_dump_write(void *info)
{
    clear_pending(&TIMERG0, WDT_MWDT0);  // task watchdog
    clear_pending(&TIMERG1, WDT_MWDT1);  // interrupt watchdog
    __real_esp_core_dump_write(info);
}
