/* ALERT receiver: station-name lookup for decoded addresses.
 *
 * Two sources, one lookup (tools/alert/V2_SPEC.md section 8): a table uploaded
 * over the console into SPI flash when one is present and valid, else the
 * table built into the image - app/alert_stations_gen.h, generated from the
 * MegaNet repository by tools/alert/gen_stations.py (never edit that file).
 * Phase A has the built-in table only; the upload functions refuse.
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
// kind may each be NULL when only the answer is wanted.
bool        ALERTSTN_Lookup(uint16_t id, char *name, uint8_t name_max, uint8_t *kind);
// "BUILTIN MegaNet:95f6f8d" or "SPI MegaNet:xxxxxxx"
const char *ALERTSTN_Source(void);
// Validates the SPI table once, at app entry.
void        ALERTSTN_Init(void);
uint16_t    ALERTSTN_Count(void);

// Console-facing upload (section 8 blob). Phase A: all refuse.
bool ALERTSTN_UploadBegin(uint32_t len, uint32_t crc32);
bool ALERTSTN_UploadWrite(uint32_t off, const uint8_t *p, uint16_t n);
bool ALERTSTN_UploadEnd(void);          // verify the CRC, activate
bool ALERTSTN_Clear(void);              // erase, revert to the built-in table

// Short label for a kind code: "RAIN", "LVL", "BATT", "REP", "SNSR", "CHK", "".
const char *ALERT_KindLabel(uint8_t kind);

#endif
