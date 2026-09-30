/* ALERT receiver: station-name lookup - see alert_stn.h.
 *
 * Copyright 2026 cdomotor-g. Apache-2.0, like the egzumer base it lives in.
 */
#ifdef ENABLE_ALERT

#include "app/alert_stn.h"
#include "app/alert_stations_gen.h"   // static tables: this must stay its only includer

// Binary search for the last site whose base_id <= id; a site covers base_id
// to base_id + 4, one 3-bit kind per offset.
static bool BuiltinFind(uint16_t id, const char **name, uint8_t *kind)
{
	int lo = 0, hi = ALERT_STATIONS_COUNT - 1, best = -1;
	while (lo <= hi) {
		const int mid = (lo + hi) / 2;
		if (gAlertSites[mid].base_id <= id) {
			best = mid;
			lo = mid + 1;
		} else {
			hi = mid - 1;
		}
	}

	if (best >= 0) {
		const AlertSite_t *s = &gAlertSites[best];
		const uint16_t off = id - s->base_id;
		if (off < 5) {
			const uint8_t k = (s->kinds >> (3 * off)) & 7u;
			if (k != ALERT_KIND_NONE) {
				*name = &gAlertNames[s->name_off];
				*kind = k;
				return true;
			}
		}
	}
	return false;
}

bool ALERTSTN_Lookup(uint16_t id, char *name, uint8_t name_max, uint8_t *kind)
{
	const char *src = "";
	uint8_t     k   = ALERT_KIND_NONE;
	const bool  hit = BuiltinFind(id, &src, &k);

	if (name && name_max) {
		uint8_t i = 0;
		while (src[i] && i + 1u < name_max) {
			name[i] = src[i];
			i++;
		}
		name[i] = 0;
	}
	if (kind)
		*kind = k;
	return hit;
}

const char *ALERTSTN_Source(void)
{
	return "BUILTIN " ALERT_STATIONS_SOURCE;
}

void ALERTSTN_Init(void)
{
	// no SPI table yet: nothing to validate
}

uint16_t ALERTSTN_Count(void)
{
	return ALERT_STATIONS_COUNT;
}

bool ALERTSTN_UploadBegin(uint32_t len, uint32_t crc32)
{
	(void)len;
	(void)crc32;
	return false;
}

bool ALERTSTN_UploadWrite(uint32_t off, const uint8_t *p, uint16_t n)
{
	(void)off;
	(void)p;
	(void)n;
	return false;
}

bool ALERTSTN_UploadEnd(void)
{
	return false;
}

bool ALERTSTN_Clear(void)
{
	return false;
}

const char *ALERT_KindLabel(uint8_t kind)
{
	static const char labels[8][5] = { "", "RAIN", "LVL", "BATT", "REP", "SNSR", "CHK", "" };
	return labels[kind & 7u];
}

#endif // ENABLE_ALERT
