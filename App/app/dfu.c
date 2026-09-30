/* Copyright 2026 ALERT-X1
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 *     Unless required by applicable law or agreed to in writing, software
 *     distributed under the License is distributed on an "AS IS" BASIS,
 *     WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *     See the License for the specific language governing permissions and
 *     limitations under the License.
 */

/* Hands-off DFU and reset robustness. See dfu.h for the overview and plan
 * sections 6/7 for the design.
 *
 * Every bootloader address and instruction the trampoline depends on was
 * re-verified against the stock 7.00.07 image (bl70007.bin, 9984 B,
 * sha256 40bab5b0..7c3f, zlib CRC32 of 0x08000000..0x080026FF == 0x22CDCECB)
 * with a Thumb disassembler, and is asserted again in CI from the same file.
 * The verified facts each step relies on are noted at that step.
 */

#include "app/dfu.h"

#include <string.h>

#include "py32f0xx.h"                 // RCC / SCB / SysTick / NVIC / core intrinsics
#include "external/printf/printf.h"   // sprintf

#if defined(ENABLE_USB)
    #include "driver/vcp.h"
#endif
#if defined(ENABLE_UART)
    #include "driver/uart.h"
#endif

// The git hash comes from CMake (App/CMakeLists.txt defines BUILD_COMMIT for
// every build); fall back so the file always compiles standalone.
#ifndef BUILD_COMMIT
    #define BUILD_COMMIT "unknown"
#endif

// ---------------------------------------------------------------------------
// No-init state
//
// One cell placed by the linker at 0x20003FE0, above _estack, in a NOLOAD region
// the startup code never clears. The stock bootloader only touches
// 0x200001A0..0x2000257F (its ZI fill) and keeps its stack top at 0x20002580, so
// this cell survives both a warm reset and the app<->bootloader hand-off; a
// power-on leaves it as undefined RAM, which the signature detects.

#define DFU_MAGIC   0xDF00B007u   // "boot into DFU on the next reset"
#define NOINIT_SIG  0xA1E70C2Du   // the cell holds our state (survived a warm reset)

typedef struct {
    uint32_t dfu_magic;      // == DFU_MAGIC when DFU entry is requested
    uint32_t dfu_magic_inv;  // == ~DFU_MAGIC (paired to reject stray RAM values)
    uint32_t sig;            // == NOINIT_SIG when the cell is ours
    uint32_t sig_inv;        // == ~NOINIT_SIG
    uint8_t  reset_reason;   // DFU_RST_*
    uint8_t  abnormal_count; // consecutive abnormal resets (saturating)
    uint8_t  pending;        // a fault/watchdog handler recorded the reason
    uint8_t  pad;
} dfu_noinit_t;

// volatile: written from the fault/SysTick handlers and read across a reset, so
// the compiler must never elide or reorder these stores.
static volatile dfu_noinit_t g_ni __attribute__((section(".noinit"), used));

// ---------------------------------------------------------------------------
// Output (same USB/UART path the ALERT telemetry uses; a no-op when no host has
// the port open with DTR asserted, exactly like alert.c's DbgSend).

static void dfu_send(const char *s)
{
#if defined(ENABLE_USB)
    VCP_SendStr(s);
#endif
#if defined(ENABLE_UART)
    UART_Send(s, strlen(s));
#endif
#if !defined(ENABLE_USB) && !defined(ENABLE_UART)
    (void)s;
#endif
}

static const char *dfu_git_hash(void)
{
    return BUILD_COMMIT;
}

// ---------------------------------------------------------------------------
// On-device bootloader guard
//
// zlib/RFC-1952 CRC32 (reflected, poly 0xEDB88320) over the whole stock
// bootloader image, computed bitwise so there is no 1 KB table to carry. ~80k
// iterations run only when a DFU jump is actually pending, never on a normal boot.

#define BL_BASE       0x08000000u
#define BL_LEN        0x2700u          // 0x08000000..0x080026FF, the whole image
#define BL_CRC_GOOD   0x22CDCECBu
#define BL_VER_ADDR   0x0800209Au
#define BL_VER_STR    "7.00.07"        // 7 chars, verified present at BL_VER_ADDR

static uint32_t bl_crc32(void)
{
    const uint8_t *p = (const uint8_t *)BL_BASE;
    uint32_t crc = 0xFFFFFFFFu;
    for (uint32_t i = 0; i < BL_LEN; i++) {
        crc ^= p[i];
        for (int k = 0; k < 8; k++)
            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
    return ~crc;
}

static bool bl_guard_ok(void)
{
    if (bl_crc32() != BL_CRC_GOOD)
        return false;
    return memcmp((const void *)BL_VER_ADDR, BL_VER_STR, sizeof(BL_VER_STR) - 1) == 0;
}

uint32_t DFU_BootloaderCrc(void)
{
    static uint32_t cached;
    static bool have;
    if (!have) { cached = bl_crc32(); have = true; }
    return cached;
}

bool DFU_BootloaderOk(void)
{
    return DFU_BootloaderCrc() == BL_CRC_GOOD
        && memcmp((const void *)BL_VER_ADDR, BL_VER_STR, sizeof(BL_VER_STR) - 1) == 0;
}

// ---------------------------------------------------------------------------
// The trampoline
//
// Reconstructs the machine state the stock bootloader expects and re-enters its
// own DFU path at 0x080013B2 (main, just past the PTT test that would otherwise
// gate DFU on a held key). It only writes RCC/SCB/NVIC registers and calls the
// bootloader's own routines; it never erases or writes flash. On any wrong turn
// the worst case is a hang, which a power cycle recovers.

typedef void (*bl_fn0_t)(void);
typedef void (*bl_fn1_t)(uint32_t);
typedef void (*bl_fn3_t)(uint32_t, uint32_t, uint32_t);

static void dfu_do_trampoline(void) __attribute__((noreturn, noinline));

static void dfu_do_trampoline(void)
{
    // 1. Quiesce the core. main() is entered with interrupts on and SysTick
    //    running; the bootloader expects neither.
    __disable_irq();
    SysTick->CTRL = 0;
    NVIC->ICER[0] = 0xFFFFFFFFu;   // one ICER/ICPR on ARMv6-M covers all 32 IRQs
    NVIC->ICPR[0] = 0xFFFFFFFFu;
    __DSB();
    __ISB();

    // 2. Pulse every peripheral reset. This detaches USB (APBRSTR1 USBDRST,
    //    bit 23) so the bootloader can re-enumerate cleanly. AHBRSTR has no
    //    flash/SRAM reset bit, so this is safe while executing from flash.
    RCC->IOPRSTR  = 0xFFFFFFFFu; RCC->IOPRSTR  = 0;
    RCC->AHBRSTR  = 0xFFFFFFFFu; RCC->AHBRSTR  = 0;
    RCC->APBRSTR1 = 0xFFFFFFFFu; RCC->APBRSTR1 = 0;
    RCC->APBRSTR2 = 0xFFFFFFFFu; RCC->APBRSTR2 = 0;

    // 3. Put SYSCLK back on HSI and turn the PLL off. The bootloader's clock
    //    init at 0x08000EC8 clears PLLON and then spins at 0x08000EFC until
    //    PLLRDY == 0, which only works if the PLL is not the current source.
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY)) { }
    RCC->CFGR &= ~RCC_CFGR_SW;                                 // SW = 0 = HSI
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI) { } // SWS_HSI == 0
    RCC->CR &= ~RCC_CR_PLLON;
    while (RCC->CR & RCC_CR_PLLRDY) { }

    // 4. Point the vector table at the bootloader.
    SCB->VTOR = BL_BASE;

    // 5. Replay the C-runtime scatter-load the bootloader's __main did before
    //    its main(). The region table at 0x080020A4 gives both records:
    //      [080020A4]=080020C4 src [080020A8]=20000000 dst [080020AC]=0001A0 len
    //      [080020B0]=080001C8 decompress func
    //      [080020B4]=080020D8 src [080020B8]=200001A0 dst [080020BC]=0023E0 len
    //      [080020C0]=08001160 zero-fill func
    //    The zero-fill covers 0x200001A0..0x2000257F exactly (up to the 0x20002580
    //    stack top). After this the bootloader's RAM globals are live and no app
    //    global may be touched (only registers, immediates and this local stack,
    //    which sits above 0x20002580 and is untouched).
    ((bl_fn3_t)0x080001C9u)(0x080020C4u, 0x20000000u, 0x000001A0u);
    ((bl_fn3_t)0x08001161u)(0x080020D8u, 0x200001A0u, 0x000023E0u);

    // 6. Replay main's own prologue (0x08001328..): the peripheral clocks it
    //    enables before doing anything else.
    RCC->IOPENR  |= 0x00000007u;   // IOP A/B/C
    RCC->APBENR2 |= 0x00004001u;   // SYSCFG + USART1
    RCC->APBENR1 |= 0x10800000u;   // USB + PWR
    RCC->AHBENR  |= 0x00000001u;   // DMA

    // 7. The four helpers main calls next: clock init (0x08000EC9), 0x08000689,
    //    0x08000749, then the USB/DFU init (0x08000549) with r0 = 0x20001DE0.
    ((bl_fn0_t)0x08000EC9u)();
    ((bl_fn0_t)0x08000689u)();
    ((bl_fn0_t)0x08000749u)();
    ((bl_fn1_t)0x08000549u)(0x20001DE0u);

    // 8. Rebuild the {r3-r7, lr} frame main pushed (6 words -> MSP = 0x20002580 -
    //    0x18 = 0x20002568) so that when main eventually returns it pops into
    //    0x080000DC ("b ."), a safe hang. The zero-fill above already cleared the
    //    frame; only the lr slot at 0x2000257C matters.
    *(volatile uint32_t *)0x2000257Cu = 0x080000DDu;

    // Set MSP and the registers main holds at 0x080013B2 (r4=0x20000020,
    // r5=0x2000, r6=0x50000800, r7=0x50000400), re-enable interrupts, and resume
    // main just past its PTT test at 0x080013B3 (Thumb). The bootloader then sets
    // DFU state 1, starts USB, lights the torch, beacons 0x0518, and boots the new
    // app itself after the last page.
    __asm volatile (
        "ldr r0, =0x20002568  \n\t"
        "msr MSP, r0          \n\t"
        "isb                  \n\t"
        "ldr r4, =0x20000020  \n\t"
        "ldr r5, =0x00002000  \n\t"
        "ldr r6, =0x50000800  \n\t"
        "ldr r7, =0x50000400  \n\t"
        "ldr r0, =0x080013B3  \n\t"
        "cpsie i              \n\t"
        "bx  r0               \n\t"
        ".ltorg               \n\t"
        ::: "r0", "r4", "r5", "r6", "r7", "memory"
    );

    __builtin_unreachable();
}

void DFU_Trampoline(void)
{
    // Cheap on every normal boot: one no-init read and out.
    if (g_ni.dfu_magic != DFU_MAGIC || g_ni.dfu_magic_inv != (uint32_t)~DFU_MAGIC)
        return;

    // Consume the request first, so a wrong turn below cannot loop back into DFU.
    g_ni.dfu_magic = 0;
    g_ni.dfu_magic_inv = 0;

    // Re-check the bootloader on the device. The magic is only ever set behind
    // this same guard, but re-checking means a corrupted magic (or a mismatched
    // bootloader) boots the app normally instead of jumping into the unknown.
    if (!bl_guard_ok())
        return;

    dfu_do_trampoline();   // never returns
}

// ---------------------------------------------------------------------------
// Reset classification and the abnormal-reset counter

void DFU_BootInit(void)
{
    bool valid = (g_ni.sig == NOINIT_SIG) && (g_ni.sig_inv == (uint32_t)~NOINIT_SIG);
    if (!valid) {
        // Power-on / brown-out: the cell was not ours. Start a clean history.
        g_ni.sig            = NOINIT_SIG;
        g_ni.sig_inv        = ~NOINIT_SIG;
        g_ni.dfu_magic      = 0;
        g_ni.dfu_magic_inv  = 0;
        g_ni.reset_reason   = DFU_RST_POR;
        g_ni.abnormal_count = 0;
        g_ni.pending        = 0;
        return;
    }

    if (g_ni.pending)
        g_ni.pending = 0;                    // a fault/watchdog handler set the reason
    else
        g_ni.reset_reason = DFU_RST_SW;      // a clean NVIC_SystemReset (user, 0x05DD, ...)
}

uint8_t DFU_ResetReason(void)   { return g_ni.reset_reason; }
uint8_t DFU_AbnormalCount(void) { return g_ni.abnormal_count; }

// ---------------------------------------------------------------------------
// Boot line

void DFU_EmitBootLine(void)
{
    static const char *const names[] = { "por", "sw", "wd", "fault" };
    uint8_t r = DFU_ResetReason();
    if (r > DFU_RST_FAULT)
        r = DFU_RST_SW;

    char line[80];
    sprintf(line, "B ver=%s rst=%s n=%u bl=%08lX\r\n",
            dfu_git_hash(), names[r], (unsigned)DFU_AbnormalCount(),
            (unsigned long)DFU_BootloaderCrc());
    dfu_send(line);
}

// ---------------------------------------------------------------------------
// DFU request / auto-heal

bool DFU_RequestDfu(void)
{
    if (!DFU_BootloaderOk())
        return false;

    // A DFU entry means someone is about to flash a fix; let the new build start
    // with a clean abnormal-reset history.
    g_ni.abnormal_count = 0;
    g_ni.sig            = NOINIT_SIG;
    g_ni.sig_inv        = ~NOINIT_SIG;
    g_ni.dfu_magic      = DFU_MAGIC;
    g_ni.dfu_magic_inv  = ~DFU_MAGIC;
    __DSB();
    NVIC_SystemReset();
    return true;   // not reached
}

bool DFU_AutostartBlocked(void) { return DFU_AbnormalCount() >= 3; }
bool DFU_AutoDfuDue(void)       { return DFU_AbnormalCount() >= 5; }

// ---------------------------------------------------------------------------
// Robustness: soft watchdog, healthy timer, fault handler

static volatile uint16_t s_uptime_10ms;
static volatile uint16_t s_wd_10ms;
static volatile bool     s_wd_armed;

static void dfu_record_reason_and_reset(uint8_t reason)
{
    g_ni.sig          = NOINIT_SIG;
    g_ni.sig_inv      = ~NOINIT_SIG;
    g_ni.reset_reason = reason;
    if (g_ni.abnormal_count < 255)
        g_ni.abnormal_count++;
    g_ni.pending = 1;
    __DSB();
    NVIC_SystemReset();
}

void DFU_RecordFaultAndReset(void)
{
    dfu_record_reason_and_reset(DFU_RST_FAULT);
}

void DFU_WatchdogArm(bool on)
{
    s_wd_armed = on;
    s_wd_10ms  = 0;
}

void DFU_WatchdogKick(void)
{
    s_wd_10ms = 0;
}

void DFU_WatchdogTick(void)
{
    if (s_uptime_10ms < 0xFFFFu)
        s_uptime_10ms++;

    // 60 s of healthy running clears the abnormal-reset history, so an old fault
    // does not eventually force DFU once the radio has settled.
    if (s_uptime_10ms == 6000u)
        g_ni.abnormal_count = 0;

    if (s_wd_armed && ++s_wd_10ms >= 500u) {   // 5 s with no kick -> the loop stalled
        s_wd_armed = false;                    // one-shot; the reset takes over
        dfu_record_reason_and_reset(DFU_RST_WD);
    }
}

// ---------------------------------------------------------------------------
// Host commands (called from App/app/uart.c)

void DFU_HostCheck(void)
{
    // Report the bootloader version bytes as read from flash, sanitised so a
    // mismatched image cannot inject control characters into the line.
    char ver[8];
    const uint8_t *v = (const uint8_t *)BL_VER_ADDR;
    for (int i = 0; i < 7; i++) {
        uint8_t c = v[i];
        ver[i] = (c >= 0x20 && c < 0x7F) ? (char)c : '?';
    }
    ver[7] = 0;

    char line[96];
    sprintf(line, "K bl crc=%08lX ver=%s ok=%u fw=%s\r\n",
            (unsigned long)DFU_BootloaderCrc(), ver,
            DFU_BootloaderOk() ? 1u : 0u, dfu_git_hash());
    dfu_send(line);
}

void DFU_HostEnter(const uint8_t *payload, uint16_t len)
{
    uint32_t magic = 0;
    if (len >= 4)
        magic = (uint32_t)payload[0] | ((uint32_t)payload[1] << 8)
              | ((uint32_t)payload[2] << 16) | ((uint32_t)payload[3] << 24);

    if (magic != DFU_HOST_MAGIC) {
        dfu_send("K dfu ok=0 err=magic\r\n");
        return;
    }
    if (!DFU_BootloaderOk()) {
        dfu_send("K dfu ok=0 err=guard\r\n");
        return;
    }

    dfu_send("K dfu ok=1\r\n");
    DFU_RequestDfu();   // arms the magic and resets (guard already confirmed)
}

void DFU_EmitAck(const char *tag, bool ok)
{
    char line[48];
    sprintf(line, "K %s ok=%u\r\n", tag, ok ? 1u : 0u);
    dfu_send(line);
}
