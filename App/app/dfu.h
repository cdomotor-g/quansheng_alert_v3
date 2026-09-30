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

/* Hands-off DFU and reset robustness for the ALERT-X1 build (plan sections 6/7).
 *
 * The stock 7.00.07 bootloader only enters DFU when PTT is held at power-on, and
 * the running firmware has no way to write flash or reach DFU. This module adds
 * a software route: ENTER_DFU (a UART/USB command, or the automatic reset-loop
 * heal) arms a magic word in a no-init RAM cell and resets; on the next boot the
 * trampoline - the very first thing main() runs - hands control to the stock
 * bootloader's own DFU entry point, but only after re-checking the bootloader
 * image on the device (CRC32 + version string). Nothing here ever erases or
 * writes flash itself, so the worst a bug can do is hang, which a power cycle
 * recovers.
 *
 * It also records why the MCU last reset (power-on / clean software reset /
 * soft-watchdog / HardFault) and counts consecutive abnormal resets, so a build
 * that crash-loops backs off its autostart and eventually heals into DFU.
 */
#ifndef APP_DFU_H
#define APP_DFU_H

#include <stdbool.h>
#include <stdint.h>

// Reset reasons reported in the boot 'B' line (plan section 8).
enum {
    DFU_RST_POR   = 0,   // power-on / brown-out: the no-init RAM was not ours
    DFU_RST_SW    = 1,   // clean software reset (NVIC_SystemReset), e.g. 0x05DD
    DFU_RST_WD    = 2,   // the soft watchdog fired (the ALERT loop stalled)
    DFU_RST_FAULT = 3,   // HardFault
};

// The magic the host must place in the 0x05E0 ENTER_DFU payload ("DFU!"). It is
// only a guard against a stray command; the real safety check is the on-device
// bootloader CRC/version guard, which is verified before any jump.
#define DFU_HOST_MAGIC  0x44465521u

// ---------------------------------------------------------------------------
// Boot path (Core/Src/main.c). DFU_Trampoline() must be the first statement of
// main(), before any peripheral setup; DFU_BootInit() runs right after it.

// If ENTER_DFU was requested and the on-device guard passes, hand control to the
// stock bootloader's DFU. Never returns in that case. Otherwise returns at once.
void DFU_Trampoline(void);

// Classify this reset from the no-init cell and manage the abnormal-reset
// counter. Safe to call once, early, before the app touches anything.
void DFU_BootInit(void);

uint8_t DFU_ResetReason(void);      // one of DFU_RST_*
uint8_t DFU_AbnormalCount(void);    // consecutive abnormal resets, saturating at 255

// The stock-bootloader guard (0x08000000..0x080026FF CRC32 == 0x22CDCECB and
// "7.00.07" at 0x0800209A). Computed once and cached.
uint32_t DFU_BootloaderCrc(void);
bool     DFU_BootloaderOk(void);

// Emit the boot 'B' line over the same USB/UART path the ALERT app uses.
void DFU_EmitBootLine(void);

// ---------------------------------------------------------------------------
// Reset-loop / autostart policy (App/app/app.c, plan section 7).

// Arm DFU for the next boot: if the bootloader guard passes and the radio is not
// transmitting, clear the abnormal counter, set the magic and reset through
// DFU_SafeReset(). Returns false (without resetting) otherwise. Used by 0x05E0
// and by the auto-heal path. Needs BK4819_Init() to have run.
bool DFU_RequestDfu(void);

// True while the radio transmits (gCurrentFunction == FUNCTION_TRANSMIT).
bool DFU_Transmitting(void);

// NVIC_SystemReset(), after taking the BK4829 out of TX (PA bias and enable off,
// REG_30 idle): the chip keeps its TX state across an MCU reset and the stock
// bootloader never touches it, so a reset that ends in DFU would otherwise leave
// a carrier on. Needs BK4819_Init() to have run.
void DFU_SafeReset(void) __attribute__((noreturn));

// True while the app should refuse to autostart the sweep (>= 3 abnormal resets).
bool DFU_AutostartBlocked(void);
// True once the abnormal count has reached the auto-DFU threshold (>= 5).
bool DFU_AutoDfuDue(void);

// ---------------------------------------------------------------------------
// Robustness hooks.

// Contract D -> B: the soft watchdog for the (blocking) ALERT loop.
void DFU_WatchdogArm(bool on);   // B: on at APP_RunAlert entry, off at exit
void DFU_WatchdogKick(void);     // B: once per main-loop pass inside APP_RunAlert

// True once DFU_WatchdogArm(true) has run this boot, i.e. APP_RunAlert was
// entered by any path; the autostart then has nothing left to start.
bool DFU_AppEntered(void);

// Called every SysTick (App/scheduler.c): services the soft watchdog and the
// 60 s "healthy" timer that forgets the abnormal-reset history.
void DFU_WatchdogTick(void);

// Called from HardFault_Handler (Core/Src/py32f071_it.c): record + reset.
void DFU_RecordFaultAndReset(void);

// ---------------------------------------------------------------------------
// Host commands (routed from App/app/uart.c).

void DFU_HostCheck(void);                                    // 0x05E1 DFU_CHECK
void DFU_HostEnter(const uint8_t *payload, uint16_t len);   // 0x05E0 ENTER_DFU

// Emit a "K <tag> ok=<0|1>" ack line for the ALERT host commands.
void DFU_EmitAck(const char *tag, bool ok);

#endif // APP_DFU_H
