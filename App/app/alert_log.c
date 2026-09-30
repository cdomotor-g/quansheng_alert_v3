/* ALERT receiver: SPI-flash decode log - see alert_log.h. Phase A stub.
 *
 * Copyright 2026 cdomotor-g. Apache-2.0, like the egzumer base it lives in.
 */
#ifdef ENABLE_ALERT

#include "app/alert_log.h"

bool ALERTLOG_Init(void)
{
	return false;
}

bool ALERTLOG_Append(const AlertRecord_t *r)
{
	(void)r;
	return false;
}

uint32_t ALERTLOG_Count(void)
{
	return 0;
}

uint32_t ALERTLOG_Capacity(void)
{
	return 0;
}

bool ALERTLOG_ReadBack(uint32_t back, AlertRecord_t *r, uint32_t *seq)
{
	(void)back;
	(void)r;
	(void)seq;
	return false;
}

uint16_t ALERTLOG_Boot(void)
{
	return 0;
}

bool ALERTLOG_Clear(void)
{
	return false;
}

bool ALERTLOG_Format(bool force)
{
	(void)force;
	return false;
}

const char *ALERTLOG_State(void)
{
	return "OFF";
}

#endif // ENABLE_ALERT
