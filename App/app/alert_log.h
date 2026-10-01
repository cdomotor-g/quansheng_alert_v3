/* ALERT receiver: the decode log, a ring of records in SPI flash.
 *
 * Region and record format: tools/alert/V2_SPEC.md section 8, and the
 * layout comment at the top of alert_log.c. 0x160000-0x1BFFFF, 96 sectors of
 * one 32-byte header slot and 127 32-byte record slots each.
 *
 * Every function is cheap except where noted: the slow things a NOR flash
 * does (a 4 KB sector erase, 40-300 ms) happen in ALERTLOG_Use once per boot
 * at most, in ALERTLOG_Append when it crosses into a sector that was not
 * pre-erased, in ALERTLOG_Idle, and in Clear/Format. Never more than one
 * erase per call, so no call blocks for more than ~300 ms (plus, once ever,
 * ~0.2 s reading a blank region through before it is first formatted).
 *
 * ALERTLOG_Init only reads; it is the log as it stands, for the console and
 * the screens. ALERTLOG_Use is "LOG is on": alert.c calls it at app entry and
 * before an append, and until then nothing here writes the region.
 */
#ifndef APP_ALERT_LOG_H
#define APP_ALERT_LOG_H

#include <stdbool.h>
#include <stdint.h>

#include "app/alert_int.h"

bool     ALERTLOG_Init(void);               // false: log unusable (region not blank and no magic)
bool     ALERTLOG_Append(const AlertRecord_t *r);   // assigns nothing; seq is internal
uint32_t ALERTLOG_Count(void);              // records currently held
uint32_t ALERTLOG_Capacity(void);
bool     ALERTLOG_ReadBack(uint32_t back, AlertRecord_t *r, uint32_t *seq); // back 0 = newest
uint16_t ALERTLOG_Boot(void);               // this boot's counter
bool     ALERTLOG_Clear(void);
bool     ALERTLOG_Format(bool force);       // force: also over foreign data (console only)
const char *ALERTLOG_State(void);           // "OK", "OFF", "FOREIGN", "ERR"

// The seq the next ALERTLOG_Append will give its record, 0 while the log is
// unusable. The CSV DEC line's seq is this when logging (V2_SPEC section 6).
uint32_t ALERTLOG_NextSeq(void);

// LOG is on: this boot uses the log. The first call per boot formats a blank
// region (one erase) or moves the boot counter on (a one-byte write; one
// erase every 96 boots); later calls only return. Until it has run,
// ALERTLOG_Boot is 0 (V2_SPEC 2: "0 if no log"). True when the log is usable.
bool     ALERTLOG_Use(void);

// Pre-erase the sector the ring will enter next, once the current one is half
// full, so the Append that crosses into it does not have to. One sector erase
// at most (40-300 ms), none when that sector already reads blank (as after a
// reboot), and only when one is due: call it at a quiet moment
// (alert_console.c does, with the squelch shut for a second).
void     ALERTLOG_Idle(void);

#endif
