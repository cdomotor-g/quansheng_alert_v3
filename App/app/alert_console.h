/* ALERT receiver: a line-based text console on the USB CDC port.
 *
 * Commands and responses: tools/alert/V2_SPEC.md section 7. It shares the
 * port with the binary 0xABCD protocol (hotflash.py, serialtool) and never
 * consumes a binary frame: how, in alert_console.c.
 */
#ifndef APP_ALERT_CONSOLE_H
#define APP_ALERT_CONSOLE_H

#ifdef ENABLE_ALERT

// Called every loop pass inside the ALERT app AND from app.c's 10 ms slice
// outside it, so the console answers in normal radio operation too. Does
// nothing while the squelch is open (up to 2 s) or for 100 ms after it shuts.
void ALERTCON_Poll(void);

#endif

#endif
