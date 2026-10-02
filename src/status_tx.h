#ifndef STATUS_TX_H_
#define STATUS_TX_H_

/* Zone state reported in byte 10 of the status frame. */
enum zone_state {
	ZONE_STATE_ERROR = 0,   /* power-up, link loss, out-of-range command */
	ZONE_STATE_NORMAL = 1,
};

/* Safe to call from any thread; the next status frame reports it. */
void status_tx_set_zone_state(enum zone_state state);

#endif
