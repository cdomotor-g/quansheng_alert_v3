/* ALERT receiver: station-name lookup - see alert_stn.h.
 *
 * The SPI table, 0x1C0000-0x1DFFFF (128 KB; why that region is free: the map
 * at the top of alert_log.c), little-endian, as gen_stations.py --blob writes it:
 *
 *   header  "ASTB", u16 version 1, u16 count, u32 names_len,
 *           u32 crc32 (zlib) of everything after the header, char source[16]
 *   sites   count x { u16 base_id, u16 kinds, u32 name_off }, sorted by base_id;
 *           kinds packs 3 bits per address base..base+4, as alert_stations_gen.h
 *   names   NUL-terminated, <= 40 chars, uppercase, no commas; name_off
 *           counts from the first of them
 *
 * Ownership, like the log's: a region whose first sector does not start with
 * "ASTB" is FOREIGN unless the first 32 bytes of all 32 sectors are 0xFF, and
 * a foreign region is never written until STN FORMAT FORCE. Every write here
 * therefore keeps "ASTB" + version at 0x1C0000: a clear, or the start of an
 * upload, erases sector 0 and writes those six bytes straight back (count and
 * the rest left 0xFF: "cleared"), and an upload may not change them. A power
 * cut mid-upload leaves a table of ours that fails its CRC, never a region
 * that looks like someone else's.
 *
 * Copyright 2026 cdomotor-g. Apache-2.0, like the egzumer base it lives in.
 */
#ifdef ENABLE_ALERT

#include <string.h>

#include "app/alert_stn.h"
#include "app/alert_stations_gen.h"   // static tables: this must stay its only includer
#include "driver/py25q16.h"

#define STN_BASE     0x1C0000u
#define STN_SIZE     0x20000u
#define STN_SECTOR   0x1000u
#define STN_VERSION  1u

typedef struct {
	char     magic[4];      // "ASTB"
	uint16_t version;       // STN_VERSION
	uint16_t count;         // sites
	uint32_t names_len;
	uint32_t crc32;         // of everything after this header
	char     source[16];    // "MegaNet:xxxxxxx", NUL-padded
} StnHdr_t;

typedef struct {
	uint16_t base_id;
	uint16_t kinds;
	uint32_t name_off;
} StnSite_t;

_Static_assert(sizeof(StnHdr_t) == 32, "V2_SPEC section 8 header");
_Static_assert(sizeof(StnSite_t) == 8, "V2_SPEC section 8 site record");

#define SITES_ADDR   (STN_BASE + sizeof(StnHdr_t))

enum { ST_UNCHECKED = 0, ST_BUILTIN, ST_SPI, ST_BAD, ST_FOREIGN };

// What sector 0 always starts with once the region is ours.
static const uint8_t kClaim[6] = { 'A', 'S', 'T', 'B', STN_VERSION, 0 };

static uint8_t  sState;
static uint16_t sCount;          // sites in the SPI table
static uint32_t sCrc;            // its body CRC
static char     sSource[20];     // "SPI " + its source
static uint32_t sUpLen;          // upload in progress: its length, else 0
static uint32_t sUpCrc;
static uint32_t sUpErased;       // bytes [0, sUpErased) erased for it

// zlib/RFC-1952 CRC32, bitwise like dfu.c's: no 1 KB table in flash.
static uint32_t Crc32Flash(uint32_t addr, uint32_t len)
{
	uint8_t  buf[64];
	uint32_t crc = 0xFFFFFFFFu;

	while (len) {
		const uint32_t n = len < sizeof(buf) ? len : sizeof(buf);
		PY25Q16_ReadBuffer(addr, buf, n);
		for (uint32_t i = 0; i < n; i++) {
			crc ^= buf[i];
			for (uint8_t k = 0; k < 8u; k++)
				crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
		}
		addr += n;
		len  -= n;
	}
	return ~crc;
}

static bool RegionBlank(void)
{
	uint8_t b[32];

	for (uint32_t a = STN_BASE; a < STN_BASE + STN_SIZE; a += STN_SECTOR) {
		PY25Q16_ReadBuffer(a, b, sizeof(b));
		for (uint8_t i = 0; i < sizeof(b); i++)
			if (b[i] != 0xFFu)
				return false;
	}
	return true;
}

static void Validate(void)
{
	StnHdr_t h;

	PY25Q16_ReadBuffer(STN_BASE, &h, sizeof(h));
	sState = ST_BUILTIN;
	if (memcmp(&h, kClaim, 4) != 0) {
		if (!RegionBlank())
			sState = ST_FOREIGN;
		return;
	}
	if (h.count == 0xFFFFu && h.names_len == 0xFFFFFFFFu)
		return;                          // cleared: the built-in by choice

	sState = ST_BAD;
	if (h.version != STN_VERSION || h.count == 0 || h.names_len > STN_SIZE)
		return;
	const uint32_t body = (uint32_t)h.count * sizeof(StnSite_t) + h.names_len;
	if (body > STN_SIZE - sizeof(h) || Crc32Flash(SITES_ADDR, body) != h.crc32)
		return;

	sCount = h.count;
	sCrc   = h.crc32;
	memcpy(sSource, "SPI ", 4);
	uint8_t i = 0;
	// it ends up in CSV fields: printable, no commas
	while (i < 15u && h.source[i] > ' ' && h.source[i] < 0x7F && h.source[i] != ',') {
		sSource[4 + i] = h.source[i];
		i++;
	}
	sSource[4 + i] = 0;
	sState = ST_SPI;
}

// Sector 0 erased and "ASTB" + version written back: ours, and empty.
static void Claim(void)
{
	PY25Q16_SectorErase(STN_BASE);
	PY25Q16_WriteBuffer(STN_BASE, kClaim, sizeof(kClaim), false);
}

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

// The same search over the SPI sites, one 8-byte read per probe.
static bool SpiFind(uint16_t id, char *name, uint8_t name_max, uint8_t *kind)
{
	StnSite_t s, hit = { 0, 0, 0 };
	int lo = 0, hi = (int)sCount - 1;
	bool found = false;

	while (lo <= hi) {
		const int mid = (lo + hi) / 2;
		PY25Q16_ReadBuffer(SITES_ADDR + (uint32_t)mid * sizeof(s), &s, sizeof(s));
		if (s.base_id <= id) {
			hit   = s;
			found = true;
			lo = mid + 1;
		} else {
			hi = mid - 1;
		}
	}
	const uint16_t off = id - hit.base_id;
	if (!found || off >= 5u)
		return false;
	const uint8_t k = (hit.kinds >> (3u * off)) & 7u;
	if (k == ALERT_KIND_NONE)
		return false;
	*kind = k;

	if (name && name_max > 1u) {
		const uint8_t n = (uint8_t)(name_max - 1u < ALERT_NAME_MAX ? name_max - 1u : ALERT_NAME_MAX);
		// the name's own NUL ends it sooner; name[n] ends a 40-char one
		PY25Q16_ReadBuffer(SITES_ADDR + (uint32_t)sCount * sizeof(StnSite_t) + hit.name_off, name, n);
		name[n] = 0;
	}
	return true;
}

bool ALERTSTN_Lookup(uint16_t id, char *name, uint8_t name_max, uint8_t *kind)
{
	uint8_t k = ALERT_KIND_NONE;
	bool    hit;

	if (name && name_max)
		name[0] = 0;
	if (sState == ST_SPI) {
		hit = SpiFind(id, name, name_max, &k);
	} else {
		const char *src = "";
		hit = BuiltinFind(id, &src, &k);
		if (name && name_max) {
			uint8_t i = 0;
			while (src[i] && i + 1u < name_max) {
				name[i] = src[i];
				i++;
			}
			name[i] = 0;
		}
	}
	if (kind)
		*kind = k;
	return hit;
}

const char *ALERTSTN_Source(void)
{
	return sState == ST_SPI ? sSource : "BUILTIN " ALERT_STATIONS_SOURCE;
}

void ALERTSTN_Init(void)
{
	if (sState == ST_UNCHECKED)
		Validate();
}

uint16_t ALERTSTN_Count(void)
{
	return sState == ST_SPI ? sCount : ALERT_STATIONS_COUNT;
}

uint32_t ALERTSTN_Crc(void)
{
	return sState == ST_SPI ? sCrc : 0;
}

const char *ALERTSTN_State(void)
{
	static const char *const names[] = { "BUILTIN", "BUILTIN", "SPI", "BAD", "FOREIGN" };
	return names[sState];
}

bool ALERTSTN_UploadBegin(uint32_t len, uint32_t crc32)
{
	ALERTSTN_Init();
	if (sState == ST_FOREIGN || len <= sizeof(StnHdr_t) || len > STN_SIZE)
		return false;
	Claim();                    // the old table is gone from here: built-in until END
	sState    = ST_BUILTIN;
	sUpLen    = len;
	sUpCrc    = crc32;
	sUpErased = STN_SECTOR;
	return true;
}

bool ALERTSTN_UploadWrite(uint32_t off, const uint8_t *p, uint16_t n)
{
	if (!sUpLen || off > sUpLen || n > sUpLen - off)
		return false;
	// The claim bytes must stay as Claim wrote them: changing them would take
	// an erase of sector 0 and could leave it looking foreign.
	for (uint32_t i = off; i < sizeof(kClaim) && i < off + n; i++)
		if (p[i - off] != kClaim[i])
			return false;
	// Each sector is erased the first time the upload reaches it: in order,
	// that is one erase per 64 STN W lines, never more than one per call.
	while (sUpErased < off + n) {
		PY25Q16_SectorErase(STN_BASE + sUpErased);
		sUpErased += STN_SECTOR;
	}
	PY25Q16_WriteBuffer(STN_BASE + off, p, n, false);
	return true;
}

uint32_t ALERTSTN_UploadLen(void)
{
	return sUpLen;
}

bool ALERTSTN_UploadEnd(void)
{
	const uint32_t len = sUpLen;

	if (!len)
		return false;
	sUpLen = 0;
	// The transfer, then the blob's own checks (Validate). On a failure the
	// region keeps its claim and the built-in stays in use.
	if (Crc32Flash(STN_BASE, len) != sUpCrc)
		return false;
	Validate();
	return sState == ST_SPI;
}

bool ALERTSTN_Format(bool force)
{
	ALERTSTN_Init();
	if (sState == ST_FOREIGN && !force)
		return false;
	sUpLen = 0;
	Claim();
	sState = ST_BUILTIN;
	return true;
}

bool ALERTSTN_Clear(void)
{
	return ALERTSTN_Format(false);
}

const char *ALERT_KindLabel(uint8_t kind)
{
	static const char labels[8][5] = { "", "RAIN", "LVL", "BATT", "REP", "SNSR", "CHK", "" };
	return labels[kind & 7u];
}

#endif // ENABLE_ALERT
