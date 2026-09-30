/* ALERT receiver: audio-pin census and MCU ADC sampler - see alert_adc.h.
 *
 * Copyright 2026 cdomotor-g. Apache-2.0, like the egzumer base it lives in.
 */
#ifdef ENABLE_ALERT

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "app/alert_adc.h"
#include "board.h"
#include "driver/bk4819.h"
#include "driver/gpio.h"
#include "driver/system.h"
#include "driver/systick.h"
#include "external/printf/printf.h"
#include "py32f071_ll_adc.h"
#include "py32f071_ll_bus.h"
#include "py32f071_ll_dac.h"
#include "py32f071_ll_dma.h"
#include "py32f071_ll_gpio.h"
#include "py32f071_ll_system.h"
#include "py32f071_ll_tim.h"

// The DAC is the voice-prompt output. With ENABLE_VOICE on, VOICE_Init owns
// it (trigger, DMA channel 3), so the bias mode is left out rather than reset
// out from under the prompts.
#ifdef ENABLE_VOICE
	#define DAC_MODES 1u
#else
	#define DAC_MODES 2u
#endif

// ---------------------------------------------------------------------------
// sampler: TIM3 -> ADC1 scan [CH4, CH9] -> DMA1 channel 1 -> IRQ

#define ADC_FS        9600u
#define NCH          2u          // scan order: 0 = CH4 (PA4), 1 = CH9 (PB1)
#define SCANS_HALF    64u         // one DMA half = one Goertzel block = 6.7 ms
#define CENSUS_BLOCKS 16u         // 1024 scans per census condition
// A stuck-open squelch must not overflow the 64-bit sums; 27 s of burst is
// far more than any qualifying one, and later blocks are simply not counted.
#define BURST_BLOCKS  4096u
#define GRAB_TIMEOUT_MS 200u      // 16 blocks + one dropped = 120 ms when all is well

static uint16_t dmaBuf[2u * SCANS_HALF * NCH];   // 512 B
static bool     sampling;

// Goertzel at 1300 (space, near enough to 1312.5), 1700 (between the tones:
// a noise reference) and 2100 Hz (mark). Q12 keeps coef * state inside int32
// for any 12-bit input over a 64-sample block (|state| < 64 * 4095 / sin w).
enum { G13 = 0, G17, G21, NTONE };
static const int32_t goertzelQ12[NTONE] = { 5401, 3623, 1598 };   // 4096 * 2cos(2 pi f / 9600)

typedef struct {
	int32_t  dc_x16;          // slow DC tracker, runs whenever the sampler does
	uint64_t pw[NTONE];       // summed block power
	uint64_t sq;              // sum of (v - ref)^2
	int32_t  sum;             // sum of (v - ref)
	uint16_t ref, mn, mx;     // ref = first sample, so the variance keeps its precision off mid-rail
} Chan_t;

enum { ACC_IDLE = 0, ACC_ARMED, ACC_RUN, ACC_DONE };

static Chan_t            chn[NCH];
static volatile uint8_t  accState;
static volatile uint16_t accBlocks;
static uint16_t          accLimit;
static bool              dcInit;     // trackers take the first sample after SamplerStart

// ---------------------------------------------------------------------------
// software AFSK demodulator, ported from the DP32G030 ALERT_AdcTick
//
// The integer arithmetic is unchanged, so tools/alert/simdemod.py still models
// it bit for bit. Two differences: the sample arrives as an argument from the
// DMA interrupt instead of being read by a 9600 Hz SysTick, and the DC tracker
// starts at the first sample instead of 2048 - PB1 sits near 780 counts, and
// a tracker walking down from mid-rail would spend the first ~100 ms of every
// burst feeding the correlators a DC step.

#define ALERT_MARK_HZ  2100u
#define ALERT_SPACE_HZ 1300u
#define MARK_INC  ((256u * ALERT_MARK_HZ  + ADC_FS / 2u) / ADC_FS)   // 56 = 2100.0 Hz
#define SPACE_INC ((256u * ALERT_SPACE_HZ + ADC_FS / 2u) / ADC_FS)   // 35 = 1312.5 Hz
#define DEMOD_SPB (ADC_FS / 300u)                                     // 32 samples per bit
#define CORR_W    16u                                                 // correlator window, 1.67 ms
// 768 bits = 2.56 s at 300 baud. A qualifying burst is at most 1.5 s.
#define DEMOD_BYTES 96u
#define DEMOD_BITS  (DEMOD_BYTES * 8u)

// quarter-wave-symmetric 32-entry sine, int8
static const int8_t sinTab[32] = {
	  0,  25,  49,  71,  90, 106, 117, 125, 127, 125, 117, 106,  90,  71,  49,  25,
	  0, -25, -49, -71, -90,-106,-117,-125,-127,-125,-117,-106, -90, -71, -49, -25
};

static inline int8_t sin256(uint8_t ph) { return sinTab[ph >> 3]; }
static inline int8_t cos256(uint8_t ph) { return sinTab[(uint8_t)(ph + 64u) >> 3]; }

static struct {
	int32_t  dc_x16;
	int32_t  s1c, s1s, s2c, s2s;
	int16_t  h1c[CORR_W], h1s[CORR_W], h2c[CORR_W], h2s[CORR_W];
	uint16_t nbits;
	uint8_t  ph1, ph2;              // tone phases, 1/256 cycle
	uint8_t  hi;
	uint8_t  phase;                 // bit clock phase 0..DEMOD_SPB-1
	uint8_t  lastBit;
	uint8_t  chan;                  // scan index being demodulated
	bool     first;
	bool     on;
	uint16_t skip;                  // samples still to ignore (amplifier settling)
	uint8_t  buf[DEMOD_BYTES];
} dm;

// When a burst has to switch the audio amplifier on (PA8) the pin sees its
// turn-on step and pop first; the census never measures that, since each of its
// PA8-on groups waits before sampling. So the demodulator ignores this much
// and seeds its DC tracker after it: 20 ms, six bits of a ~60-bit preamble.
#define PA8_SETTLE_SAMPLES (ADC_FS / 50u)

static inline void DemodSample(int32_t raw)
{
	if (dm.skip) {
		dm.skip--;
		return;
	}
	if (dm.first) {
		dm.first  = false;
		dm.dc_x16 = raw << 4;
	}

	// DC removal (slow tracker) and scaling to ~8 bits
	dm.dc_x16 += ((raw << 4) - dm.dc_x16) >> 7;
	int32_t x = (raw - (dm.dc_x16 >> 4)) >> 3;

	// two quadrature correlators with a sliding window, in int16. A 12-bit
	// sample can sit up to 4095 counts off the tracker (a step the ~13 ms DC
	// tracker has not caught up with yet), which would make |x| 511 and wrap
	// the products; clamped, 256 * 127 = 32512 fits. tools/alert/simdemod.py
	// clamps the same way.
	if (x > 255)
		x = 255;
	else if (x < -256)
		x = -256;
	const int16_t p1c = (int16_t)(x * cos256(dm.ph1)), p1s = (int16_t)(x * sin256(dm.ph1));
	const int16_t p2c = (int16_t)(x * cos256(dm.ph2)), p2s = (int16_t)(x * sin256(dm.ph2));
	dm.ph1 = (uint8_t)(dm.ph1 + MARK_INC);
	dm.ph2 = (uint8_t)(dm.ph2 + SPACE_INC);

	dm.s1c += p1c - dm.h1c[dm.hi]; dm.h1c[dm.hi] = p1c;
	dm.s1s += p1s - dm.h1s[dm.hi]; dm.h1s[dm.hi] = p1s;
	dm.s2c += p2c - dm.h2c[dm.hi]; dm.h2c[dm.hi] = p2c;
	dm.s2s += p2s - dm.h2s[dm.hi]; dm.h2s[dm.hi] = p2s;
	if (++dm.hi >= CORR_W) dm.hi = 0;

	const int32_t a1 = dm.s1c >> 6, b1 = dm.s1s >> 6, a2 = dm.s2c >> 6, b2 = dm.s2s >> 6;
	const uint8_t bit = ((a1 * a1 + b1 * b1) > (a2 * a2 + b2 * b2)) ? 1u : 0u;   // mark = 1

	// bit clock recovery: pull the phase towards the transitions
	if (bit != dm.lastBit) {
		if (dm.phase < DEMOD_SPB / 2u) dm.phase -= dm.phase / 4u;
		else                           dm.phase += (DEMOD_SPB - dm.phase) / 4u;
		dm.lastBit = bit;
	}

	if (++dm.phase >= DEMOD_SPB) {
		dm.phase = 0;
		// sample point: the correlator window (~half a bit) already delays the
		// decision, so the current one is the middle of the bit
		if (dm.nbits < DEMOD_BITS) {
			const uint8_t m = (uint8_t)(0x80u >> (dm.nbits & 7u));
			if (bit) dm.buf[dm.nbits >> 3] |= m;
			else     dm.buf[dm.nbits >> 3] &= (uint8_t)~m;
			dm.nbits++;
		}
	}
}

// ---------------------------------------------------------------------------
// interrupt side

static uint64_t BlockPower(int32_t q1, int32_t q2, int32_t coef)
{
	// |X|^2 = q1^2 + q2^2 - 2cos(w) q1 q2
	const int64_t p = (int64_t)q1 * q1 + (int64_t)q2 * q2 - (((int64_t)q1 * q2 * coef) >> 12);
	return (p > 0) ? (uint64_t)p : 0u;
}

static void AccumulateChannel(Chan_t *k, const uint16_t *p)
{
	int32_t a1 = 0, a2 = 0, b1 = 0, b2 = 0, c1 = 0, c2 = 0;   // Goertzel states, 1300/1700/2100
	int32_t  dc  = k->dc_x16;
	int32_t  sum = k->sum;
	uint64_t sq  = k->sq;
	uint16_t mn  = k->mn, mx = k->mx;

	for (uint32_t i = 0; i < SCANS_HALF; i++) {
		const int32_t v = p[i * NCH];
		if (v < mn) mn = (uint16_t)v;
		if (v > mx) mx = (uint16_t)v;
		const int32_t d = v - (int32_t)k->ref;
		sum += d;
		sq  += (uint32_t)(d * d);

		// DC off first: PB1 sits ~1270 counts below mid-rail, and that much DC
		// leaking through a 64-sample window would bury a small tone
		dc += ((v << 4) - dc) >> 7;
		const int32_t x = v - (dc >> 4);

		int32_t t;
		t = x + ((goertzelQ12[G13] * a1) >> 12) - a2; a2 = a1; a1 = t;
		t = x + ((goertzelQ12[G17] * b1) >> 12) - b2; b2 = b1; b1 = t;
		t = x + ((goertzelQ12[G21] * c1) >> 12) - c2; c2 = c1; c1 = t;
	}

	k->dc_x16 = dc;
	k->sum = sum;
	k->sq  = sq;
	k->mn  = mn;
	k->mx  = mx;
	// Power is summed per block, not carried across blocks: incoherent
	// averaging stays valid when the tone is a few Hz off the bin, and the
	// state never grows past one block's worth.
	k->pw[G13] += BlockPower(a1, a2, goertzelQ12[G13]);
	k->pw[G17] += BlockPower(b1, b2, goertzelQ12[G17]);
	k->pw[G21] += BlockPower(c1, c2, goertzelQ12[G21]);
}

static void TrackChannel(Chan_t *k, const uint16_t *p)
{
	int32_t dc = k->dc_x16;
	for (uint32_t i = 0; i < SCANS_HALF; i++)
		dc += (((int32_t)p[i * NCH] << 4) - dc) >> 7;
	k->dc_x16 = dc;
}

static void ProcessHalf(const uint16_t *p)
{
	if (dm.on) {
		const uint16_t *s = p + dm.chan;
		for (uint32_t i = 0; i < SCANS_HALF; i++)
			DemodSample(s[i * NCH]);
	}

	if (!dcInit) {
		dcInit = true;
		for (uint32_t c = 0; c < NCH; c++)
			chn[c].dc_x16 = (int32_t)p[c] << 4;
	}

	switch (accState) {
		case ACC_ARMED:
			// This half may hold samples from before whatever change armed the
			// grab. Dropping all of it is the plan's "drop the first 8 scans",
			// rounded up so that every Goertzel block stays whole.
			for (uint32_t c = 0; c < NCH; c++)
				TrackChannel(&chn[c], p + c);
			accState = ACC_RUN;
			return;

		case ACC_RUN:
			for (uint32_t c = 0; c < NCH; c++) {
				Chan_t *k = &chn[c];
				if (accBlocks == 0) {
					k->ref = p[c];
					k->mn  = p[c];
					k->mx  = p[c];
				}
				AccumulateChannel(k, p + c);
			}
			accBlocks++;
			if (accLimit && accBlocks >= accLimit)
				accState = ACC_DONE;
			return;

		default:
			for (uint32_t c = 0; c < NCH; c++)
				TrackChannel(&chn[c], p + c);
			return;
	}
}

// The startup file's weak default for this vector is Default_Handler (a spin).
// Nothing else in the tree uses DMA1 channel 1: 2 = UART RX, 3 = voice DAC,
// 4/5 = SPI flash, 7 = backlight.
void DMA1_Channel1_IRQHandler(void)
{
	// HT: the first half is complete while the DMA fills the second; TC: the
	// reverse. Flags are cleared before the work so an edge that lands during
	// it is not lost.
	if (LL_DMA_IsActiveFlag_HT1(DMA1)) {
		LL_DMA_ClearFlag_HT1(DMA1);
		ProcessHalf(&dmaBuf[0]);
	}
	if (LL_DMA_IsActiveFlag_TC1(DMA1)) {
		LL_DMA_ClearFlag_TC1(DMA1);
		ProcessHalf(&dmaBuf[SCANS_HALF * NCH]);
	}
	if (LL_DMA_IsActiveFlag_TE1(DMA1))
		LL_DMA_ClearFlag_TE1(DMA1);   // the channel has stopped itself; grabs time out
}

// ---------------------------------------------------------------------------
// peripheral setup

// BOARD_ADC_Init was written against an ADC fresh out of reset, and it
// calibrates, which this part only allows with ADON clear (LL_ADC_StartCalibration).
// So reset the peripheral and run it again: that is the boot state exactly,
// whatever this file changed (scan mode, rank 2, DMA, trigger, CH4/CH9 timing).
static void AdcRestore(void)
{
	LL_ADC_Disable(ADC1);
	LL_APB1_GRP2_ForceReset(LL_APB1_GRP2_PERIPH_ADC1);
	LL_APB1_GRP2_ReleaseReset(LL_APB1_GRP2_PERIPH_ADC1);
	BOARD_ADC_Init();
}

static void SamplerStart(void)
{
	if (sampling)
		return;

	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
	LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

	// TIM3: 48 MHz / 5000 = 9600 Hz; each update is a TRGO, each TRGO one scan.
	// The timers run at the core clock here (voice.c: 48 MHz / 4 / 1500 = 8 kHz).
	// From reset, so no leftover prescaler waits for an update to load.
	LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_TIM3);
	LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_TIM3);
	LL_TIM_SetPrescaler(TIM3, 0);
	LL_TIM_SetAutoReload(TIM3, SystemCoreClock / ADC_FS - 1u);
	LL_TIM_SetTriggerOutput(TIM3, LL_TIM_TRGO_UPDATE);

	// ADC: configured with ADON clear. On this F1-style ADC, rewriting CR2 while
	// ADON is set can itself start a conversion, and one stray conversion would
	// swap CH4 and CH9 in every scan from then on.
	LL_ADC_Disable(ADC1);
	LL_ADC_SetSequencersScanMode(ADC1, LL_ADC_SEQ_SCAN_ENABLE);
	LL_ADC_REG_SetTriggerSource(ADC1, LL_ADC_REG_TRIG_EXT_TIM3_TRGO);
	LL_ADC_REG_SetContinuousMode(ADC1, LL_ADC_REG_CONV_SINGLE);
	LL_ADC_REG_SetSequencerLength(ADC1, LL_ADC_REG_SEQ_SCAN_ENABLE_2RANKS);
	LL_ADC_REG_SetSequencerDiscont(ADC1, LL_ADC_REG_SEQ_DISCONT_DISABLE);
	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_4);
	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_9);
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_4, LL_ADC_SAMPLINGTIME_41CYCLES_5);
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_9, LL_ADC_SAMPLINGTIME_41CYCLES_5);
	LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_UNLIMITED);
	LL_ADC_REG_StartConversionExtTrig(ADC1, LL_ADC_REG_TRIG_EXT_RISING);

	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
	LL_SYSCFG_SetDMARemap(DMA1, LL_DMA_CHANNEL_1, LL_SYSCFG_DMA_MAP_ADC1);
	LL_DMA_ConfigTransfer(DMA1, LL_DMA_CHANNEL_1,
	                      LL_DMA_DIRECTION_PERIPH_TO_MEMORY | LL_DMA_MODE_CIRCULAR |
	                      LL_DMA_PERIPH_NOINCREMENT | LL_DMA_MEMORY_INCREMENT |
	                      LL_DMA_PDATAALIGN_HALFWORD | LL_DMA_MDATAALIGN_HALFWORD |
	                      LL_DMA_PRIORITY_HIGH);
	LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_1, LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA));
	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)dmaBuf);
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, sizeof(dmaBuf) / sizeof(dmaBuf[0]));
	LL_DMA_ClearFlag_GI1(DMA1);
	LL_DMA_EnableIT_HT(DMA1, LL_DMA_CHANNEL_1);
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);

	dcInit = false;
	__COMPILER_BARRIER();
	// Below the SPI-flash DMA (1) and SysTick (0), above USB (3): a half is
	// overwritten 6.7 ms after it completes, and the work takes ~0.3 ms.
	NVIC_ClearPendingIRQ(DMA1_Channel1_IRQn);
	NVIC_SetPriority(DMA1_Channel1_IRQn, 2);
	NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);

	LL_ADC_Enable(ADC1);   // ADON 0 -> 1 powers up; it does not convert
	SYSTICK_DelayUs(10);   // tSTAB
	sampling = true;
	LL_TIM_EnableCounter(TIM3);
}

static void SamplerStop(void)
{
	if (!sampling)
		return;

	LL_TIM_DisableCounter(TIM3);
	SYSTICK_DelayUs(20);   // let a scan already under way land (2 x 4.5 us)

	NVIC_DisableIRQ(DMA1_Channel1_IRQn);
	LL_DMA_DisableIT_HT(DMA1, LL_DMA_CHANNEL_1);
	LL_DMA_DisableIT_TC(DMA1, LL_DMA_CHANNEL_1);
	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
	LL_DMA_ClearFlag_GI1(DMA1);
	NVIC_ClearPendingIRQ(DMA1_Channel1_IRQn);
	__COMPILER_BARRIER();
	sampling = false;

	AdcRestore();

	// TIM3 back to its boot state: reset and unclocked. DMA1 stays clocked
	// (the SPI flash uses it) and channel 1's remap value is its reset value.
	LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_TIM3);
	LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_TIM3);
	LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_TIM3);
}

// Start accumulating on both channels at the next whole block after this one.
// The ISR only touches chn[] in ACC_RUN, so going through ACC_IDLE makes the
// reset safe without masking interrupts.
static void AccArm(uint16_t limit)
{
	accState = ACC_IDLE;
	__COMPILER_BARRIER();
	for (uint32_t c = 0; c < NCH; c++) {
		const int32_t dc = chn[c].dc_x16;
		memset(&chn[c], 0, sizeof(chn[c]));
		chn[c].dc_x16 = dc;
	}
	accBlocks = 0;
	accLimit  = limit;
	__COMPILER_BARRIER();
	accState = ACC_ARMED;
}

static bool dacOn;

// PA4 bias: DAC1 channel 1 unbuffered at mid-scale. Behind a coupling
// capacitor PA4 otherwise has no defined DC level, and leakage can park it
// against a rail and clip the audio. Off means reset and unclocked, as at boot.
static void Pa4Dac(bool on)
{
#ifdef ENABLE_VOICE
	(void)on;
#else
	if (on == dacOn)
		return;
	dacOn = on;
	if (on) {
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_DAC1);
		LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_DAC1);
		LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_DAC1);
		LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_4, LL_GPIO_MODE_ANALOG);
		// From reset: no trigger, no DMA, no wave, output to the pin. With the
		// trigger off, DHR moves to the output one clock after each write.
		LL_DAC_SetOutputBuffer(DAC1, LL_DAC_CHANNEL_1, LL_DAC_OUTPUT_BUFFER_DISABLE);
		LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_1, 0x800);
		LL_DAC_Enable(DAC1, LL_DAC_CHANNEL_1);
		LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_1, 0x800);
	} else {
		LL_DAC_Disable(DAC1, LL_DAC_CHANNEL_1);
		LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_DAC1);
		LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_DAC1);
		LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_DAC1);
	}
#endif
}

static bool Pa8IsOn(void)
{
	return LL_GPIO_IsOutputPinSet(GPIOA, LL_GPIO_PIN_8) != 0;
}

static void Pa8Set(bool on)
{
	if (on) GPIO_EnableAudioPath();
	else    GPIO_DisableAudioPath();
}

// ---------------------------------------------------------------------------
// levels

// 100 * log10(v), i.e. dB x10 against 1: log2 from the top bit's position,
// the fraction from a 16-step table with linear interpolation (< 0.1 dB).
static const uint16_t log2Frac[17] = {
	0, 22, 44, 63, 82, 100, 118, 134, 150, 165, 179, 193, 207, 220, 232, 244, 256
};

static int16_t DB10(uint64_t v)
{
	int32_t  s = 0;
	uint32_t m;

	if (v == 0)
		return 0;
	while (v >> 32) {
		v >>= 1;
		s++;
	}
	m = (uint32_t)v;
	while (m >= 512u) { m >>= 1; s++; }
	while (m < 256u)  { m <<= 1; s--; }

	const uint32_t f  = m - 256u;
	const uint32_t i  = f >> 4;
	const int32_t  l  = (int32_t)log2Frac[i] + (int32_t)(((uint32_t)(log2Frac[i + 1] - log2Frac[i]) * (f & 15u)) >> 4);
	const int32_t  q8 = (8 + s) * 256 + l;                 // log2(v), Q8, >= 0 for v >= 1
	return (int16_t)((q8 * 30103 + 128000) / 256000);      // x 100 log10(2)
}

typedef struct {
	uint16_t mean, pp;
	int16_t  rms;
	int16_t  g[NTONE];
} Level_t;

// Read one channel's sums. Only called with the ISR out of the way: after
// ACC_DONE, or with the sampler stopped.
static void Measure(uint32_t c, uint32_t blocks, Level_t *lv)
{
	const Chan_t  *k = &chn[c];
	const uint32_t n = blocks * SCANS_HALF;

	if (n == 0) {
		lv->mean = 0;
		lv->pp   = 0;
		lv->rms  = ALERTADC_LEVEL_NONE;
		for (uint32_t t = 0; t < NTONE; t++)
			lv->g[t] = ALERTADC_LEVEL_NONE;
		return;
	}
	lv->mean = (uint16_t)((int32_t)k->ref + k->sum / (int32_t)n);
	lv->pp   = (uint16_t)(k->mx - k->mn);
	// n^2 * variance = n * sum(d^2) - (sum d)^2, kept whole so no division
	// throws the precision away; dividing by n^2 is a subtraction in dB.
	{
		const uint64_t a = (uint64_t)n * k->sq;
		const uint64_t b = (uint64_t)((int64_t)k->sum * k->sum);
		lv->rms = (int16_t)(DB10(a > b ? a - b : 1u) - 2 * DB10(n));
	}
	for (uint32_t t = 0; t < NTONE; t++)
		lv->g[t] = (int16_t)(DB10(k->pw[t]) - DB10(blocks));
}

static AlertAdcEmit_t gEmit;

static void EmitLevel(uint32_t c, bool bias, bool pa, const char *src, const Level_t *lv)
{
	char line[112];

	if (!gEmit)
		return;
	snprintf(line, sizeof(line), "P ch=%u b=%u pa=%u src=%s mean=%u pp=%u rms=%d g13=%d g17=%d g21=%d\r\n",
	         c ? 9u : 4u, bias ? 1u : 0u, pa ? 1u : 0u, src, (unsigned)lv->mean, (unsigned)lv->pp,
	         (int)lv->rms, (int)lv->g[G13], (int)lv->g[G17], (int)lv->g[G21]);
	gEmit(line);
}

// ---------------------------------------------------------------------------
// choice

static uint8_t choicePin;
static bool    choicePa8;
static int16_t idleTab[3][2] = {   // census F level per pin and PA8, for LastBurstEnergy
	{ ALERTADC_LEVEL_NONE, ALERTADC_LEVEL_NONE },
	{ ALERTADC_LEVEL_NONE, ALERTADC_LEVEL_NONE },
	{ ALERTADC_LEVEL_NONE, ALERTADC_LEVEL_NONE },
};
static int16_t lastG13 = ALERTADC_LEVEL_NONE;
static int16_t lastG21 = ALERTADC_LEVEL_NONE;

static void ApplyChoice(uint8_t pin, bool pa8)
{
	if (pin > ALERTADC_PIN_PB1)
		pin = ALERTADC_PIN_NONE;
	choicePin = pin;
	choicePa8 = (pin != ALERTADC_PIN_NONE) && pa8;
	Pa4Dac(pin == ALERTADC_PIN_PA4B);
}

void ALERTADC_SetChoice(uint8_t pin, bool pa8)
{
	if (!sampling)
		ApplyChoice(pin, pa8);
}

uint8_t ALERTADC_ChoicePin(void) { return choicePin; }
bool    ALERTADC_ChoicePa8(void) { return choicePa8; }
bool    ALERTADC_IsSampling(void) { return sampling; }

// ---------------------------------------------------------------------------
// census

enum { SRC_M = 0, SRC_F, SRC_T13, SRC_T21, NSRC };
static const char *const srcName[NSRC] = { "M", "F", "T13", "T21" };

// Accumulate CENSUS_BLOCKS whole blocks on both channels, sampling the RSSI
// while waiting if asked. False if the sampler stopped delivering, so a dead
// DMA cannot hold the census past its 4 s.
static bool Grab(int16_t *rssiMin)
{
	AccArm(CENSUS_BLOCKS);
	for (uint32_t ms = 0; accState != ACC_DONE; ms++) {
		if (ms >= GRAB_TIMEOUT_MS) {
			accState = ACC_IDLE;
			return false;
		}
		SYSTEM_DelayMs(1);
		if (rssiMin && (ms % 5u) == 0) {
			const int16_t r = BK4819_GetRSSI_dBm();
			if (r < *rssiMin)
				*rssiMin = r;
		}
	}
	__COMPILER_BARRIER();
	return true;
}

// Digital read of a pin with its pull-up and then its pull-down. A net that
// follows the pulls is open or AC-coupled; one that does not is driven.
static void PinTest(GPIO_TypeDef *port, uint32_t pin, uint8_t *up, uint8_t *down)
{
	const uint32_t mode = LL_GPIO_GetPinMode(port, pin);
	const uint32_t pull = LL_GPIO_GetPinPull(port, pin);

	LL_GPIO_SetPinPull(port, pin, LL_GPIO_PULL_UP);
	LL_GPIO_SetPinMode(port, pin, LL_GPIO_MODE_INPUT);
	SYSTEM_DelayMs(5);
	*up = LL_GPIO_IsInputPinSet(port, pin) ? 1u : 0u;
	LL_GPIO_SetPinPull(port, pin, LL_GPIO_PULL_DOWN);
	SYSTEM_DelayMs(5);
	*down = LL_GPIO_IsInputPinSet(port, pin) ? 1u : 0u;

	LL_GPIO_SetPinMode(port, pin, mode);
	LL_GPIO_SetPinPull(port, pin, pull);
}

static int16_t RssiMin(uint32_t n)
{
	int16_t m = BK4819_GetRSSI_dBm();
	while (--n) {
		SYSTEM_DelayMs(5);
		const int16_t r = BK4819_GetRSSI_dBm();
		if (r < m)
			m = r;
	}
	return m;
}

// The audio source for one condition. The tones are the key-beep path:
// PlayTone switches REG_30 to AF DAC + discriminator + TX DSP only - no PLL/VCO,
// no RX link, no PA gain - and ExitTxMute lets the TX DSP tone reach the AF
// output. Nothing here touches REG_36 (PA bias) or the PA-enable GPIO, so no
// RF can leave the radio. PlaySingleTone and EnableTXLink are never used:
// both set REG_30 PA gain.
static void SetSource(uint32_t src)
{
	switch (src) {
		case SRC_M:
			BK4819_SetAF(BK4819_AF_MUTE);
			SYSTEM_DelayMs(10);
			break;
		case SRC_F:
			BK4819_SetAF(BK4819_AF_FM);
			SYSTEM_DelayMs(10);
			break;
		default:
			BK4819_PlayTone(src == SRC_T13 ? 1300u : 2100u, true);
			BK4819_ExitTxMute();
			SYSTEM_DelayMs(20);
			break;
	}
}

typedef struct {
	uint8_t pin;
	bool    pa;
	int16_t margin;
} Pick_t;

// Pass: under tone f the f energy is >= 10 dB over MUTE, for both tones, and
// >= 10 dB over the other tone's energy. The margin is the smallest of the
// four. g[src][0] is 1300 Hz, g[src][1] is 2100 Hz.
static void Consider(uint8_t pin, bool pa, int16_t g[NSRC][2], Pick_t *best)
{
	int32_t m = (int32_t)g[SRC_T13][0] - g[SRC_M][0];
	int32_t v;

	v = (int32_t)g[SRC_T21][1] - g[SRC_M][1];   if (v < m) m = v;
	v = (int32_t)g[SRC_T13][0] - g[SRC_T13][1]; if (v < m) m = v;
	v = (int32_t)g[SRC_T21][1] - g[SRC_T21][0]; if (v < m) m = v;

	const int16_t idle = (g[SRC_F][0] > g[SRC_F][1]) ? g[SRC_F][0] : g[SRC_F][1];
	idleTab[pin - 1u][pa ? 1 : 0] = idle;

	if (m < 100)
		return;
	// Quiet wins: with the amplifier on, the speaker plays every burst.
	if (best->pin == ALERTADC_PIN_NONE || (best->pa && !pa) || (best->pa == pa && m > best->margin)) {
		best->pin    = pin;
		best->pa     = pa;
		best->margin = (int16_t)m;
	}
}

bool ALERTADC_Census(AlertAdcEmit_t emit)
{
	static const char *const pinName[4] = { "none", "PA4", "PA4B", "PB1" };
	char     line[96];
	Pick_t   best     = { ALERTADC_PIN_NONE, false, 0 };
	int16_t  floorOn  = 0x7FFF;
	bool     ok       = true;
	uint8_t  u4, d4, u9, d9;

	gEmit = emit;
	ALERTADC_Shutdown();   // a burst left open, or a bias from an earlier choice

	const bool     pa8Was = Pa8IsOn();
	const uint16_t reg71  = BK4819_ReadRegister(BK4819_REG_71);

	// Every group starts from the state the key beep leaves: RX on, tones off.
	// The first group would otherwise see the modem's REG_70 and the rest not.
	BK4819_TurnsOffTones_TurnsOnRX();
	BK4819_SetAF(BK4819_AF_FM);
	Pa8Set(false);
	SYSTEM_DelayMs(20);
	const int16_t floorOff = RssiMin(8);

	// Amplifier off, so the pulls cannot click through the speaker if PA4 is
	// in the audio path.
	PinTest(GPIOA, LL_GPIO_PIN_4, &u4, &d4);
	PinTest(GPIOB, LL_GPIO_PIN_1, &u9, &d9);
	if (gEmit) {
		snprintf(line, sizeof(line), "PT PA4 u%u d%u PB1 u%u d%u\r\n",
		         (unsigned)u4, (unsigned)d4, (unsigned)u9, (unsigned)d9);
		gEmit(line);
	}

	for (uint8_t i = 0; i < 3; i++)
		idleTab[i][0] = idleTab[i][1] = ALERTADC_LEVEL_NONE;

	SamplerStart();
	for (uint32_t dac = 0; dac < DAC_MODES && ok; dac++) {
		// the bias steps PA4, so never with the amplifier on
		Pa8Set(false);
		Pa4Dac(dac != 0);

		for (uint32_t pa = 0; pa < 2 && ok; pa++) {
			int16_t g[NCH][NSRC][2];

			Pa8Set(pa != 0);
			SYSTEM_DelayMs(20);

			for (uint32_t src = 0; src < NSRC; src++) {
				Level_t lv;

				SetSource(src);
				ok = Grab(src == SRC_F ? &floorOn : NULL);
				if (src >= SRC_T13)
					BK4819_EnterTxMute();
				for (uint32_t c = 0; c < NCH; c++) {
					Measure(c, ok ? CENSUS_BLOCKS : 0u, &lv);
					EmitLevel(c, dac != 0, pa != 0, srcName[src], &lv);
					g[c][src][0] = lv.g[G13];
					g[c][src][1] = lv.g[G21];
				}
				if (!ok)
					break;
			}
			// back to RX for the next group's F condition (and for the caller)
			BK4819_TurnsOffTones_TurnsOnRX();
			if (!ok)
				break;

			Consider(dac ? ALERTADC_PIN_PA4B : ALERTADC_PIN_PA4, pa != 0, g[0], &best);
			// PB1 is judged with the bias off only: the choice has no PB1-with-bias.
			if (!dac)
				Consider(ALERTADC_PIN_PB1, pa != 0, g[1], &best);
		}
	}
	SamplerStop();

	// The chosen bias (if any) goes on while the amplifier is still off.
	Pa8Set(false);
	if (!ok)
		best.pin = ALERTADC_PIN_NONE;
	ApplyChoice(best.pin, best.pa);
	BK4819_WriteRegister(BK4819_REG_71, reg71);
	BK4819_SetAF(BK4819_AF_FM);
	Pa8Set(pa8Was);

	if (gEmit) {
		snprintf(line, sizeof(line), "AUD pin=%s pa=%u floor_on=%d floor_off=%d\r\n",
		         pinName[choicePin], choicePa8 ? 1u : 0u,
		         (int)(floorOn == 0x7FFF ? 0 : floorOn), (int)floorOff);
		gEmit(line);
	}
	return choicePin != ALERTADC_PIN_NONE;
}

// ---------------------------------------------------------------------------
// bursts

static bool burstPa8Was;
static bool burstPa8Forced;   // BurstStart switched PA8 on; only that is undone
static bool burstOpen;
static bool lastPa;           // PA8 during the last burst, for LastBurstEnergy

void ALERTADC_BurstStart(void)
{
	if (burstOpen || sampling || choicePin == ALERTADC_PIN_NONE)
		return;

	// Normally already on since the census; this only matters after a
	// Shutdown that the caller did not follow with a census or SetChoice.
	Pa4Dac(choicePin == ALERTADC_PIN_PA4B);
	burstPa8Was    = Pa8IsOn();
	burstPa8Forced = choicePa8 && !burstPa8Was;
	if (burstPa8Forced)
		Pa8Set(true);

	memset(&dm, 0, sizeof(dm));
	dm.chan  = (choicePin == ALERTADC_PIN_PB1) ? 1u : 0u;
	dm.first = true;
	dm.skip  = burstPa8Forced ? PA8_SETTLE_SAMPLES : 0u;
	dm.on    = true;

	AccArm(BURST_BLOCKS);
	burstOpen = true;
	SamplerStart();
}

uint16_t ALERTADC_BurstStop(uint8_t *buf, uint16_t maxbits)
{
	Level_t  lv[NCH];
	uint16_t n;

	if (!burstOpen)
		return 0;

	SamplerStop();
	burstOpen = false;
	dm.on     = false;
	accState  = ACC_IDLE;

	const uint32_t blocks = accBlocks;
	const bool     pa     = Pa8IsOn();
	for (uint32_t c = 0; c < NCH; c++)
		Measure(c, blocks, &lv[c]);
	// Undo only what BurstStart did: the caller may have switched PA8 itself
	// (MONITOR) while the burst was open, and that must stand.
	if (burstPa8Forced)
		Pa8Set(false);
	burstPa8Forced = false;
	lastPa = pa;

	const uint32_t cc = (choicePin == ALERTADC_PIN_PB1) ? 1u : 0u;
	lastG13 = lv[cc].g[G13];
	lastG21 = lv[cc].g[G21];

	if (blocks) {
		for (uint32_t c = 0; c < NCH; c++)
			EmitLevel(c, dacOn, pa, "BURST", &lv[c]);
	}

	n = dm.nbits;
	if (n > maxbits)
		n = maxbits;
	if (buf && n)
		memcpy(buf, dm.buf, (n + 7u) / 8u);
	return buf ? n : 0u;
}

void ALERTADC_LastBurstEnergy(int16_t *g13, int16_t *g21, int16_t *idle)
{
	if (g13)
		*g13 = lastG13;
	if (g21)
		*g21 = lastG21;
	if (idle)
		// against the census idle for the PA8 state the burst was actually
		// measured in (MONITOR may hold it on when the choice did not need it),
		// the same (pin, bias, pa) key the P src=BURST line reports
		*idle = (choicePin == ALERTADC_PIN_NONE) ? ALERTADC_LEVEL_NONE
		                                         : idleTab[choicePin - 1u][lastPa ? 1 : 0];
}

void ALERTADC_Shutdown(void)
{
	const bool wasSampling = sampling;

	SamplerStop();   // restores the ADC itself when it was running
	if (burstOpen) {
		burstOpen = false;
		if (burstPa8Forced)
			Pa8Set(false);
		burstPa8Forced = false;
	}
	dm.on    = false;
	accState = ACC_IDLE;
	Pa4Dac(false);
	// Even when idle: cheap, and it makes "the ADC is as BOARD_ADC_Init left
	// it" true on return whatever happened before.
	if (!wasSampling)
		AdcRestore();
}

#endif // ENABLE_ALERT
