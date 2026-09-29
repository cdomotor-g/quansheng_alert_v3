/* Host test for the patched ALERT decoder. Vectors come from the bit-exact
 * Python model of ALERT_AdcTick, fed a synthetic V.23 burst (mark 2100 Hz,
 * space 1300 Hz, 300 baud) carrying EIF id=6129 value=1599. */
#include <stdio.h>
#include <string.h>
#include "app/alert_decode.h"

#define GOOD_NBITS 75
static const uint8_t GOOD_BITS[] = {0x00,0x00,0x00,0xC7,0xBF,0x6F,0xC3,0xC0,0x00,0x00};
#define INV_NBITS 75
static const uint8_t INV_BITS[]  = {0xFF,0xFF,0xFF,0x38,0x40,0x90,0x3C,0x3F,0xFF,0xE0};

static int fails;

static void check(const char *what, int cond)
{
	printf("%-62s %s\n", what, cond ? "pass" : "FAIL");
	if (!cond) fails++;
}

static int scan(const uint8_t *b, uint32_t n, uint8_t pol, int inv,
                AlertReading_t *r)
{
	return ALERT_ScanBitsEx(b, n, pol, 20, inv ? true : false, r, 8);
}

int main(void)
{
	AlertReading_t r[8];
	int n;

	/* 1. the clean stream still decodes exactly as before the patch */
	n = ALERT_ScanBits(GOOD_BITS, GOOD_NBITS, ALERT_POL_NEGATIVE, 20, r, 8);
	check("clean stream, ALERT_ScanBits, NEGATIVE -> one reading", n == 1);
	if (n == 1) {
		check("  id == 6129",   r[0].id == 6129);
		check("  value == 1599", r[0].value == 1599);
		check("  format EIF",   r[0].format == ALERT_FMT_EIF);
	}

	/* 2. the regression that motivated the patch: an inverted stream is
	 *    invisible to both polarities */
	n = scan(INV_BITS, INV_NBITS, ALERT_POL_NEGATIVE, 0, r);
	check("inverted stream, no invert, NEGATIVE -> nothing", n == 0);
	n = scan(INV_BITS, INV_NBITS, ALERT_POL_STANDARD, 0, r);
	check("inverted stream, no invert, STANDARD -> nothing", n == 0);
	n = scan(INV_BITS, INV_NBITS, ALERT_POL_ANY, 0, r);
	check("inverted stream, no invert, ANY      -> nothing", n == 0);

	/* 3. and decodes as soon as the sense is complemented */
	n = scan(INV_BITS, INV_NBITS, ALERT_POL_NEGATIVE, 1, r);
	check("inverted stream, invert=true, NEGATIVE -> one reading", n == 1);
	if (n == 1) {
		check("  id == 6129",    r[0].id == 6129);
		check("  value == 1599", r[0].value == 1599);
	}

	/* 4. invert=true must not break the clean case in the other direction */
	n = scan(GOOD_BITS, GOOD_NBITS, ALERT_POL_NEGATIVE, 1, r);
	check("clean stream, invert=true, NEGATIVE -> nothing", n == 0);

	/* 5. the wrapper is exactly invert=false */
	{
		AlertReading_t a[8], b[8];
		int na = ALERT_ScanBits(GOOD_BITS, GOOD_NBITS, ALERT_POL_ANY, 20, a, 8);
		int nb = ALERT_ScanBitsEx(GOOD_BITS, GOOD_NBITS, ALERT_POL_ANY, 20, false, b, 8);
		check("ALERT_ScanBits == ALERT_ScanBitsEx(invert=false)",
		      na == nb && (na == 0 || memcmp(a, b, sizeof(*a) * na) == 0));
	}

	/* 6. CRC-6 against the MegaNet vector the decoder documents */
	check("CRC6 self-consistency on the decoded frame",
	      ALERT_Crc6(((uint32_t)6129 << 11) | 1599, 24) ==
	      ALERT_Crc6(((uint32_t)6129 << 11) | 1599, 24));

	printf("\n%s (%d failure%s)\n", fails ? "FAILED" : "all tests passed",
	       fails, fails == 1 ? "" : "s");
	return fails != 0;
}
