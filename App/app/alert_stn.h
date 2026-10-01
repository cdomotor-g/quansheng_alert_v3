/* ALERT receiver: station-name lookup for decoded addresses.
 *
 * Two sources, one lookup (tools/alert/V2_SPEC.md section 8): a table uploaded
 * over the console into SPI flash (0x1C0000-0x1DFFFF) when one is present and
 * valid, else the table built into the image - app/alert_stations_gen.h,
 * generated from the MegaNet repository by tools/alert/gen_stations.py (never
 * edit that file). The SPI table is the blob `gen_stations.py --blob` writes;
 * its layout is described in alert_stn.c.
 *
 * Names are copied out rather than pointed at: an SPI-flash table has no
 * address the caller could hold on to.
 */
#ifndef APP_ALERT_STN_H
#define APP_ALERT_STN_H

#include <stdbool.h>
#include <stdint.h>

enum {
	ALERT_KIND_NONE  = 0,
	ALERT_KIND_RAIN  = 1,
	ALERT_KIND_LEVEL = 2,
	ALERT_KIND_BATT  = 3,
	ALERT_KIND_REP   = 4,
	ALERT_KIND_OTHER = 5,
	ALERT_KIND_CHECK = 6,
};

#define ALERT_NAME_MAX 40u     // longest station name either source holds

// Look up an ALERT address. True when it is in the table. The name is copied
// into name[name_max] (truncated, always NUL-terminated; "" on a miss) and
// *kind set to an ALERT_KIND_* code (ALERT_KIND_NONE on a miss). name and
// kind may each be NULL when only the answer is wanted. With an SPI table
// this is ~12 8-byte SPI reads and one name read, well under a millisecond.
bool        ALERTSTN_Lookup(uint16_t id, char *name, uint8_t name_max, uint8_t *kind);
// "BUILTIN MegaNet:95f6f8d" or "SPI MegaNet:xxxxxxx"
const char *ALERTSTN_Source(void);
// Validates the SPI table once per boot (its CRC over up to 128 KB, ~0.2 s);
// later calls return at once. The app calls it at entry; Lookup, Source and
// Count call it themselves, so the console outside the app sees the table too.
void        ALERTSTN_Init(void);
uint16_t    ALERTSTN_Count(void);        // sites in the table in use

// Console-facing upload (section 8 blob). BEGIN's crc32 is zlib's CRC32 of
// the whole blob, header included - the transfer check; END then also checks
// the blob's own header and body CRC before the table is used. A blob is at
// most 131,040 bytes: the region's last 32 hold its end mark (alert_stn.c).
// From BEGIN on, lookups use the built-in table. Writes erase each 4 KB
// sector the first time they reach it, and one that starts past the sectors
// reached so far is refused (false), so a write of up to 4 KB erases once at
// most (~40-300 ms).
bool ALERTSTN_UploadBegin(uint32_t len, uint32_t crc32);
bool ALERTSTN_UploadWrite(uint32_t off, const uint8_t *p, uint16_t n);
bool ALERTSTN_UploadEnd(void);          // verify the CRC, activate
bool ALERTSTN_Clear(void);              // erase, revert to the built-in table
uint32_t ALERTSTN_UploadLen(void);      // the upload in progress: its length, 0 if none

// ALERTSTN_Clear, and with force also over a region that holds someone
// else's data (console STN FORMAT FORCE), like ALERTLOG_Format.
bool        ALERTSTN_Format(bool force);
// "SPI" (table in use), "BUILTIN" (region blank or cleared), "BAD" (a table
// of ours that failed its checks, built-in in use), "FOREIGN" (not ours:
// left alone, uploads refused until STN FORMAT FORCE).
const char *ALERTSTN_State(void);
uint32_t    ALERTSTN_Crc(void);          // the SPI table's body CRC32, 0 for the built-in

// Short label for a kind code: "RAIN", "LVL", "BATT", "REP", "SNSR", "CHK", "".
const char *ALERT_KindLabel(uint8_t kind);

#endif
