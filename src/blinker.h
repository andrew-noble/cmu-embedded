#ifndef BLINKER_H_
#define BLINKER_H_

enum blinker_mode {
	BLINKER_OFF,
	BLINKER_LEFT,
	BLINKER_RIGHT,
	BLINKER_HAZARD,
};

/* Nonblocking: updates the requested mode and wakes the blinker thread. */
void blinker_set_mode(enum blinker_mode mode);

#endif
