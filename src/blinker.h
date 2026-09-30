#ifndef BLINKER_H_
#define BLINKER_H_

#include <stdint.h>

enum blinker_mode {
	BLINKER_OFF,
	BLINKER_LEFT,
	BLINKER_RIGHT,
	BLINKER_HAZARD,
};

/* Nonblocking: updates the requested mode and wakes the blinker thread. */
void blinker_set_mode(enum blinker_mode mode);

/* Call for each valid normal-operation command, using button press edges.
 * Toggle a side, then cancel after a same-direction turn returns to neutral. */
void blinker_update_turn(uint8_t pressed, int16_t steering);

#endif
