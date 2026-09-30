/* ALERT receiver: audio-pin census and MCU ADC sampler (PY32F071, UV-K5 V3).
 *
 * The FSK engine may never slice this signal, so the second route is to find
 * the receiver's audio on an ADC pin and demodulate it in software. Only two
 * pins are candidates: PA4 (ADC_IN4, also DAC_OUT1, the voice-prompt output,
 * so it should sit somewhere in the speaker path) and PB1 (ADC_IN9, a steady
 * ~0.63 V of unknown purpose). See tools/alert/X1_PLAN.md section 5.
 *
 * Hardware used, all otherwise idle in this build:
 *   TIM3          9600 Hz update, TRGO starts one ADC scan
 *   ADC1          two-rank scan per trigger: rank 1 = CH4 (PA4), rank 2 = CH9 (PB1)
 *   DMA1 ch 1     circular, 2 x 64 scans (512 B), half/full interrupts
 *   DAC1 ch 1     optional mid-rail bias on PA4 (not when ENABLE_VOICE owns it)
 * TIM3 only runs while the census or a burst is being sampled; the ADC is put
 * back exactly as BOARD_ADC_Init() leaves it every time sampling stops, so the
 * battery reading works between bursts.
 *
 * Lines emitted (CRLF, via the callback handed to ALERTADC_Census):
 *   P ch=<4|9> b=<0|1> pa=<0|1> src=<M|F|T13|T21|BURST> mean= pp= rms= g13= g17= g21=
 *       mean, pp: ADC counts. rms: 100*log10(variance in counts^2), i.e. dB x10.
 *       gNN: mean Goertzel power per 64-sample block at NN00 Hz, dB x10 against 1.
 *       b = DAC bias on PA4, pa = PA8 audio amplifier. BURST lines come from
 *       ALERTADC_BurstStop, one per channel.
 *   PT PA4 u<0|1> d<0|1> PB1 u<0|1> d<0|1>    digital read with pull-up / pull-down
 *   AUD pin=<none|PA4|PA4B|PB1> pa=<0|1> floor_on=<dBm> floor_off=<dBm>
 */
#ifndef APP_ALERT_ADC_H
#define APP_ALERT_ADC_H

#include <stdbool.h>
#include <stdint.h>

#ifdef ENABLE_ALERT

enum {
	ALERTADC_PIN_NONE = 0,
	ALERTADC_PIN_PA4  = 1,   // PA4, DAC off (high impedance)
	ALERTADC_PIN_PA4B = 2,   // PA4 with DAC1 unbuffered at 0x800 as a mid-rail bias
	ALERTADC_PIN_PB1  = 3,
};

// An energy that could not be measured (no census yet, or a burst too short
// to fill one 64-sample block).
#define ALERTADC_LEVEL_NONE ((int16_t)-32768)

typedef void (*AlertAdcEmit_t)(const char *line);   // B passes its DbgSend

// Run the audio-pin census (§5). Blocking, <= 4 s. Emits P / PT / AUD lines via emit.
// Leaves the BK4819 in RX with AF=FM, tones off, REG_71 restored; the caller re-arms the modem afterwards.
// PA8 is left as it was on entry. The emit callback is kept for the P BURST lines.
// Returns true if a pin passed; the chosen pin/PA8 setting is remembered internally
// (a failed census sets the choice to none).
bool ALERTADC_Census(AlertAdcEmit_t emit);
void ALERTADC_SetChoice(uint8_t pin, bool pa8);     // restore a persisted choice (pin: 0 none, 1 PA4, 2 PA4 with DAC bias, 3 PB1)
uint8_t ALERTADC_ChoicePin(void);
bool    ALERTADC_ChoicePa8(void);

// Squelch open: start TIM3/ADC/DMA sampling of the chosen pin and the software AFSK demodulator
// (tones 2100/1300 Hz, 300 baud; port of the old ALERT_AdcTick, now fed from the DMA IRQ).
// Does nothing when the choice is none. Switches PA8 on for the burst if the choice needs it;
// with PA4B the DAC bias is already on (it stays on from the census or SetChoice until Shutdown,
// so it has settled before the burst begins).
void ALERTADC_BurstStart(void);
// Squelch closed: stop sampling. Returns number of demodulated bits (one per ALERT bit, 300 baud)
// copied to buf (MSB-first per byte, ALERT_GetBit order), at most maxbits.
// PA8 goes back to its state at BurstStart. Emits two P ... src=BURST lines (one per pin) if a
// census set emit and at least one 64-sample block (6.7 ms after the first dropped one) was taken.
uint16_t ALERTADC_BurstStop(uint8_t *buf, uint16_t maxbits);
// Burst confirmation helper (§5): energy at 1300/2100 during the last burst vs idle, dB x10.
// idle is the census F (AF=FM, squelch shut) level for the chosen pin and PA8 setting,
// the larger of its 1300 and 2100 Hz energies. Any of them may be ALERTADC_LEVEL_NONE.
void ALERTADC_LastBurstEnergy(int16_t *g13, int16_t *g21, int16_t *idle);
// Leave everything as BOARD_ADC_Init() configured it (battery measurement keeps working).
// Also switches the DAC bias off and PA8 back if a burst was still open.
void ALERTADC_Shutdown(void);

// True while TIM3 is triggering the ADC. BOARD_ADC_GetBatteryInfo() must not
// run then (UART 0x0529 would spin on EOS against a timer-triggered sequence).
bool ALERTADC_IsSampling(void);

#endif

#endif
