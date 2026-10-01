/* ALERT receiver: SPI-flash decode log - see alert_log.h.
 *
 * Where it lives. Every PY25Q16 address in the tree, grepped 2026-10-01
 * (PY25Q16_ calls, eeprom_compat's map, and every 0x1xxxxx constant):
 *
 *   0x000000-0x009FFF  channels, names, attributes, VFOs      eeprom_compat.c, settings.c,
 *                      + foxhunt cfg 0x0090E0-0x0090E6          misc.c, radio.c, foxhunt.c
 *   0x00A000-0x00A177  settings sector: FM 0x00A028, welcome/  settings.c, fm.c, spectrum.c,
 *                      callsign 0x00A0C8/D8, spectrum 0x00A148, welcome.c, foxhunt.c
 *                      ALERT cfg 0x00A170
 *   0x010000-0x0101FF  calibration                             settings.c, radio.c
 *   0x011000-0x011407  boot logo                               ui/welcome.c
 *   0x14C000-0x14CFFF  voice clip index (CN 0x14C000, EN       audio.c (ENABLE_VOICE; off in
 *                      0x14C800)                                the RescueOps ALERT build)
 *   0x14D000 + offset  voice clips                             audio.c, same
 *   0x160000-0x1BFFFF  this log (96 sectors)                   here
 *   0x1C0000-0x1DFFFF  ALERT station table                     alert_stn.c
 *   0x1E0000-0x1E7FFF  RX/TX log                               rxtx_log.c (off in RescueOps)
 *
 * No code addresses 0x160000-0x1DFFFF except through one door: audio.c reads
 * a clip at 0x14D000 + Offset with only a sanity bound (Offset <= 0x0B0000,
 * Size <= 0x019000), which on paper reaches 0x215FFF. Where the factory voice
 * data really ends is a property of the radio's flash, not of the code, and
 * cannot be proven from here. That is what the FOREIGN rule is for: a region
 * with no ALOG magic that is not blank is left alone until LOG FORMAT FORCE.
 *
 * Format (V2_SPEC section 8). 96 sectors of 4 KB, each 128 slots of 32 bytes:
 * slot 0 is the sector header, slots 1-127 records, filled in order. The ring
 * moves one sector at a time; entering a sector erases it, which drops the
 * oldest 127 records.
 *
 *   header  "ALOG", version, flags, boot, first seq, gen, crc16, commit byte,
 *           then a 12-byte boot bitmap. gen grows by one per sector opened, so
 *           the head is the largest gen - no ambiguity when the ring wraps.
 *   record  AlertRecord_t (24), seq (4), crc16 over those, pad, commit byte.
 *
 * Commit order: the 31 bytes go first with the commit byte still 0xFF, then
 * the commit byte alone. A power cut between the two leaves a slot that is
 * neither blank nor committed: it is skipped by readers and never reused, and
 * the seq it would have had goes to the next record. Headers work the same way.
 *
 * Boot counter: the head header's boot plus the number of bitmap bits cleared
 * since. ALERTLOG_Use clears one bit once per boot (a one-byte write, no
 * erase); a sector whose 96 bits are used gets a successor with the count in
 * its header. The counter only moves on boots that use the log: with LOG OFF
 * nothing here writes at all (ALERTLOG_Init only reads), and the boot is 0.
 *
 * LOG CLEAR and FORMAT open a new sector flagged CLEAR: records older than
 * its first seq no longer count. One erase instead of 96, and seq keeps
 * growing, so a host never sees a number twice.
 *
 * The flash driver blocks on everything (DMA with a busy-wait for >= 16
 * bytes, polling below that) and PY25Q16_WriteBuffer rewrites a whole sector
 * if a write needs a 0 -> 1 bit. Every write here goes to bytes known to be
 * 0xFF, so it only ever programs: a record is two short page programs.
 *
 * Copyright 2026 cdomotor-g. Apache-2.0, like the egzumer base it lives in.
 */
#ifdef ENABLE_ALERT

#include <stddef.h>
#include <string.h>

#include "app/alert_log.h"
#include "driver/crc.h"
#include "driver/py25q16.h"

#define LOG_BASE     0x160000u
#define LOG_SECTORS  96u
#define LOG_SECTOR   0x1000u
#define SLOT_SIZE    32u
#define SLOTS        (LOG_SECTOR / SLOT_SIZE)      // 128, slot 0 the header
#define RECS         (SLOTS - 1u)                  // 127 records a sector
// Kept whatever the ring's phase: one sector is always about to be erased.
#define CAPACITY     ((LOG_SECTORS - 1u) * RECS)   // 12,065
#define LOG_VERSION  1u
#define COMMIT       0xA5u
#define HDR_CLEAR    0x01u                         // flags: nothing before `first` counts
#define BUMP_BYTES   12u

typedef struct {
	char     magic[4];              // "ALOG"
	uint8_t  version;               // LOG_VERSION
	uint8_t  flags;                 // HDR_CLEAR
	uint16_t boot;                  // boot counter when the sector was opened
	uint32_t first;                 // seq of its first record
	uint32_t gen;                   // +1 per sector opened: the head has the largest
	uint16_t crc;                   // CRC_Calculate of the 16 bytes above
	uint8_t  pad;                   // 0xFF
	uint8_t  commit;                // COMMIT, written last
	uint8_t  bumps[BUMP_BYTES];     // one bit cleared per later boot
} LogHdr_t;

typedef struct {
	AlertRecord_t rec;
	uint32_t      seq;
	uint16_t      crc;              // CRC_Calculate of rec and seq
	uint8_t       pad;              // 0xFF
	uint8_t       commit;           // COMMIT, written last
} LogRec_t;

_Static_assert(sizeof(LogHdr_t) == SLOT_SIZE, "the header fills slot 0");
_Static_assert(sizeof(LogRec_t) == SLOT_SIZE, "one record per slot");

enum { ST_OFF = 0, ST_OK, ST_FOREIGN, ST_ERR, ST_BLANK };

static uint8_t  sState;
static bool     sInit;          // ALERTLOG_Init has run this boot
static bool     sUsed;          // ... and so has ALERTLOG_Use (or a Format): sBoot is this boot's
static bool     sNextErased;    // sector sHead + 1 was erased this boot, ready to enter
static bool     sEraseDue;      // ... and it is time to do that (ALERTLOG_Idle)
static uint8_t  sHead;          // the sector the next record goes to
static uint8_t  sSlot;          // its next slot to try, 1..SLOTS (SLOTS: full)
static uint16_t sBoot;
static uint32_t sGen;           // gen of the next sector opened
static uint32_t sHeadFirst;     // first seq of the head sector
static uint32_t sNext;          // seq of the next record
static uint32_t sOldest;        // oldest seq still held
// ReadBack's last sector and the seqs it holds, [sCacheFirst, sCacheEnd).
// sCacheEnd 0: none. Anything that moves the ring clears it (Scan).
static uint8_t  sCacheSec;
static uint32_t sCacheFirst;
static uint32_t sCacheEnd;

static uint32_t SlotAddr(uint8_t sec, uint8_t slot)
{
	return LOG_BASE + (uint32_t)sec * LOG_SECTOR + (uint32_t)slot * SLOT_SIZE;
}

static uint8_t NextSector(uint8_t sec)
{
	return (uint8_t)((sec + 1u) % LOG_SECTORS);
}

static bool AllFF(const void *p, uint8_t n)
{
	const uint8_t *b = p;
	while (n--)
		if (*b++ != 0xFFu)
			return false;
	return true;
}

// True for a committed header of this version. *h is filled either way.
static bool HdrRead(uint8_t sec, LogHdr_t *h)
{
	PY25Q16_ReadBuffer(SlotAddr(sec, 0), h, sizeof(*h));
	return memcmp(h->magic, "ALOG", 4) == 0 && h->version == LOG_VERSION &&
	       h->commit == COMMIT && h->crc == CRC_Calculate(h, offsetof(LogHdr_t, crc));
}

// True for a committed record, whatever its seq. *r is filled either way.
static bool RecRead(uint8_t sec, uint8_t slot, LogRec_t *r)
{
	PY25Q16_ReadBuffer(SlotAddr(sec, slot), r, sizeof(*r));
	return r->commit == COMMIT && r->crc == CRC_Calculate(r, offsetof(LogRec_t, crc));
}

static bool SlotBlank(uint8_t sec, uint8_t slot)
{
	LogRec_t r;
	PY25Q16_ReadBuffer(SlotAddr(sec, slot), &r, sizeof(r));
	return AllFF(&r, sizeof(r));
}

// Every byte, not the sector heads Scan samples: ~1.5 ms per 4 KB sector.
bool ALERT_FlashBlank(uint32_t addr, uint32_t len)
{
	uint8_t b[64];

	while (len) {
		const uint8_t n = (uint8_t)(len < sizeof(b) ? len : sizeof(b));
		PY25Q16_ReadBuffer(addr, b, n);
		if (!AllFF(b, n))
			return false;
		addr += n;
		len  -= n;
	}
	return true;
}

// Read all 96 headers (32 bytes each, a few ms): the head is the largest gen,
// the oldest record the smallest first seq unless a CLEAR sector says
// otherwise, and a region with no header at all is blank or foreign. Then the
// head's first blank slot by binary search - slots fill in order, so the
// blank ones are a suffix - and the newest committed record before it.
static void Scan(void)
{
	LogHdr_t h;
	bool     any = false, magic = false, dirty = false;
	uint32_t maxGen = 0, minFirst = 0, clearAt = 0;

	sCacheEnd = 0;
	for (uint8_t sec = 0; sec < LOG_SECTORS; sec++) {
		if (!HdrRead(sec, &h)) {
			// An uncommitted header of ours (the power went while a sector was
			// being opened) is not someone else's data.
			if (memcmp(h.magic, "ALOG", 4) == 0)
				magic = true;
			else if (!AllFF(&h, sizeof(h)))
				dirty = true;
			continue;
		}
		if (!any || h.gen > maxGen) {
			maxGen     = h.gen;
			sHead      = sec;
			sHeadFirst = h.first;
		}
		if (!any || h.first < minFirst)
			minFirst = h.first;
		if ((h.flags & HDR_CLEAR) && h.first > clearAt)
			clearAt = h.first;
		any = true;
	}

	if (!any) {
		sState = (magic || !dirty) ? ST_BLANK : ST_FOREIGN;
		return;
	}

	sGen    = maxGen + 1u;
	sOldest = minFirst > clearAt ? minFirst : clearAt;

	uint8_t lo = 1, hi = SLOTS;
	while (lo < hi) {
		const uint8_t mid = (uint8_t)((lo + hi) / 2u);
		if (SlotBlank(sHead, mid))
			hi = mid;
		else
			lo = (uint8_t)(mid + 1u);
	}
	sSlot = lo;

	sNext = sHeadFirst;
	for (uint8_t s = lo; s-- > 1u;) {
		LogRec_t r;
		if (RecRead(sHead, s, &r)) {
			sNext = r.seq + 1u;
			break;
		}
	}
	sState = ST_OK;
}

// Make `sec` the head: erase it (unless ALERTLOG_Idle already has), write its
// header, commit byte last. Whatever it held - the oldest records in the
// ring - is gone, and Scan works out the new oldest.
static bool Open(uint8_t sec, uint8_t flags)
{
	LogHdr_t h;
	const uint32_t addr = SlotAddr(sec, 0);

	if (!(sNextErased && sec == NextSector(sHead)))
		PY25Q16_SectorErase(addr);
	sNextErased = false;
	sEraseDue   = false;

	memset(&h, 0xFF, sizeof(h));
	memcpy(h.magic, "ALOG", 4);
	h.version = LOG_VERSION;
	h.flags   = flags;
	h.boot    = sBoot;
	h.first   = sNext;
	h.gen     = sGen;
	h.crc     = CRC_Calculate(&h, offsetof(LogHdr_t, crc));
	PY25Q16_WriteBuffer(addr, &h, offsetof(LogHdr_t, commit), false);
	h.commit = COMMIT;
	PY25Q16_WriteBuffer(addr + offsetof(LogHdr_t, commit), &h.commit, 1, false);

	Scan();
	if (sState != ST_OK || sHead != sec) {
		sState = ST_ERR;
		return false;
	}
	return true;
}

// This boot's number into sBoot: the head's boot plus the bitmap bits used
// since, plus one. Returns the first bitmap byte with a bit left, BUMP_BYTES
// when all 96 are used, 0xFF when the head's header does not read back.
static uint8_t BootNow(LogHdr_t *h)
{
	uint8_t used = 0, spare = BUMP_BYTES;

	if (!HdrRead(sHead, h))
		return 0xFFu;
	for (uint8_t i = 0; i < BUMP_BYTES; i++) {
		for (uint8_t m = 1; m; m = (uint8_t)(m << 1))
			if (!(h->bumps[i] & m))
				used++;
		if (spare == BUMP_BYTES && h->bumps[i])
			spare = i;
	}
	sBoot = (uint16_t)(h->boot + used + 1u);
	return spare;
}

// This boot's number, kept: one more bit of the head's bitmap. The lowest set
// bit of the first byte that has one goes to 0 - a program, never an erase.
static void Bump(void)
{
	LogHdr_t      h;
	const uint8_t spare = BootNow(&h);

	if (spare == 0xFFu) {
		sState = ST_ERR;
	} else if (spare == BUMP_BYTES) {
		// all 96 used: the next sector carries the count in its header
		(void)Open(NextSector(sHead), 0);
	} else {
		const uint8_t v = h.bumps[spare] & (uint8_t)(h.bumps[spare] - 1u);
		PY25Q16_WriteBuffer(SlotAddr(sHead, 0) + offsetof(LogHdr_t, bumps) + spare, &v, 1, false);
	}
}

bool ALERTLOG_Init(void)
{
	// Once per boot, whoever asks first (app entry or the console), and it
	// only reads: LOG OFF leaves the region alone. ALERTLOG_Use writes.
	if (!sInit) {
		sInit = true;
		Scan();
	}
	return sState == ST_OK;
}

bool ALERTLOG_Use(void)
{
	(void)ALERTLOG_Init();
	// The boot counter moves once per power-up, not once per visit to the app.
	if (!sUsed) {
		sUsed = true;
		if (sState == ST_BLANK)
			(void)ALERTLOG_Format(false);
		else if (sState == ST_OK)
			Bump();
	}
	return sState == ST_OK;
}

bool ALERTLOG_Append(const AlertRecord_t *r)
{
	LogRec_t rec;

	if (sState != ST_OK)
		return false;

	// A slot that is not blank - a torn write from before a power cut - or a
	// record that does not read back is skipped; three in a row and the log
	// gives up (ERR) rather than chew through the sector.
	for (uint8_t tries = 0; tries < 3u; tries++) {
		if (sSlot >= SLOTS && !Open(NextSector(sHead), 0))
			return false;

		const uint8_t  slot = sSlot++;
		const uint32_t addr = SlotAddr(sHead, slot);

		PY25Q16_ReadBuffer(addr, &rec, sizeof(rec));
		if (!AllFF(&rec, sizeof(rec)))
			continue;
		rec.rec = *r;
		rec.seq = sNext;
		rec.crc = CRC_Calculate(&rec, offsetof(LogRec_t, crc));
		// pad and commit are still 0xFF from the blank read
		PY25Q16_WriteBuffer(addr, &rec, offsetof(LogRec_t, commit), false);
		rec.commit = COMMIT;
		PY25Q16_WriteBuffer(addr + offsetof(LogRec_t, commit), &rec.commit, 1, false);

		if (RecRead(sHead, slot, &rec) && rec.seq == sNext) {
			sNext++;
			if (sSlot > SLOTS / 2u && !sNextErased)
				sEraseDue = true;
			return true;
		}
	}
	sState = ST_ERR;
	return false;
}

void ALERTLOG_Idle(void)
{
	if (!sEraseDue || sState != ST_OK)
		return;
	sEraseDue = false;
	sNextErased = true;
	// sNextErased does not survive a reboot, but the erase it stood for does:
	// a few ms of reads instead of erasing a blank sector again
	const uint32_t addr = SlotAddr(NextSector(sHead), 0);
	if (!ALERT_FlashBlank(addr, LOG_SECTOR)) {
		PY25Q16_SectorErase(addr);
		Scan();             // that sector held the oldest records
	}
}

uint32_t ALERTLOG_Count(void)
{
	if (sState != ST_OK)
		return 0;
	// Until the pre-erase, one sector more than CAPACITY can be held; the
	// extra ones are the next to go, so they are not offered.
	const uint32_t n = sNext - sOldest;
	return n < CAPACITY ? n : CAPACITY;
}

uint32_t ALERTLOG_Capacity(void)
{
	return sState == ST_OK ? CAPACITY : 0;
}

uint32_t ALERTLOG_NextSeq(void)
{
	return sState == ST_OK ? sNext : 0;
}

// Find the sector holding `want` and cache it. The walk goes back through the
// headers from the head, or from the cached sector when `want` is older than
// it: a newest-first reader reads one header per sector crossed, an
// oldest-first dump re-walks from the head once per sector (95 headers at
// worst, a few ms).
static bool Locate(uint32_t want)
{
	LogHdr_t h;
	uint8_t  sec;
	uint32_t first, end;

	if (sCacheEnd && want >= sCacheFirst && want < sCacheEnd)
		return true;

	if (sCacheEnd && want < sCacheFirst) {
		sec   = sCacheSec;
		first = sCacheFirst;
	} else {
		sec   = sHead;
		first = sHeadFirst;
	}
	end = sNext;            // only kept when the head itself holds it

	for (uint8_t k = 0; want < first; k++) {
		if (k >= LOG_SECTORS)
			return false;
		sec = (uint8_t)((sec + LOG_SECTORS - 1u) % LOG_SECTORS);
		if (!HdrRead(sec, &h) || h.first > first)
			continue;       // blank, or not older than where the walk is
		end   = first;
		first = h.first;
	}
	sCacheSec   = sec;
	sCacheFirst = first;
	sCacheEnd   = end;
	return true;
}

bool ALERTLOG_ReadBack(uint32_t back, AlertRecord_t *r, uint32_t *seq)
{
	LogRec_t rec;

	if (back >= ALERTLOG_Count())
		return false;
	const uint32_t want = sNext - 1u - back;
	if (!Locate(want))
		return false;

	// Straight to its slot, unless torn slots skipped at write time pushed it on.
	for (uint32_t slot = 1u + (want - sCacheFirst); slot < SLOTS; slot++) {
		if (!RecRead(sCacheSec, (uint8_t)slot, &rec)) {
			if (AllFF(&rec, sizeof(rec)))
				break;
			continue;
		}
		if (rec.seq == want) {
			if (r)
				*r = rec.rec;
			if (seq)
				*seq = want;
			return true;
		}
		if (rec.seq > want)
			break;
	}
	return false;
}

uint16_t ALERTLOG_Boot(void)
{
	return (sState == ST_OK && sUsed) ? sBoot : 0;
}

bool ALERTLOG_Clear(void)
{
	(void)ALERTLOG_Init();
	return sState == ST_OK && ALERTLOG_Format(false);
}

// A CLEAR sector after the head when there is a ring (one erase), else sector
// 0 over a blank region - or, forced, over someone else's data, whose other
// 95 sectors are then erased one by one as the ring reaches them.
bool ALERTLOG_Format(bool force)
{
	LogHdr_t h;
	uint8_t  sec;

	(void)ALERTLOG_Init();
	if (sState == ST_FOREIGN && !force)
		return false;

	if (sState == ST_OK || sState == ST_ERR) {
		sec = NextSector(sHead);
		// Not used yet this boot (LOG OFF): the new header carries this
		// boot's number rather than a bit of the old head's bitmap
		if (!sUsed && sState == ST_OK)
			(void)BootNow(&h);
	} else {
		// Blank going by the sector heads (Scan). Before the first write, every
		// byte: data whose sector heads happen to be 0xFF is still someone's.
		if (!force && !ALERT_FlashBlank(LOG_BASE, (uint32_t)LOG_SECTORS * LOG_SECTOR)) {
			sState = ST_FOREIGN;
			return false;
		}
		sec   = 0;
		sNext = 1;
		sGen  = 0;
		sNextErased = false;
	}
	sUsed = true;
	if (!sBoot)
		sBoot = 1;
	return Open(sec, HDR_CLEAR);
}

const char *ALERTLOG_State(void)
{
	static const char *const names[] = { "OFF", "OK", "FOREIGN", "ERR", "OFF" };
	return names[sState];
}

#endif // ENABLE_ALERT
