/* ALERT receiver: the decode log, a ring of records in SPI flash.
 *
 * Region and record format: tools/alert/V2_SPEC.md section 8. Phase A is a
 * stub: the log is never usable (ALERTLOG_Init returns false, State "OFF"),
 * so every caller already takes the no-log path it will need anyway when the
 * region holds someone else's data.
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

#endif
