/* ALERT receiver: USB CDC text console - see alert_console.h. Phase A stub.
 *
 * Copyright 2026 cdomotor-g. Apache-2.0, like the egzumer base it lives in.
 */
#ifdef ENABLE_ALERT

#include "app/alert_console.h"

void ALERTCON_Poll(void)
{
	// nothing yet: the binary protocol keeps the port to itself
}

#endif // ENABLE_ALERT
