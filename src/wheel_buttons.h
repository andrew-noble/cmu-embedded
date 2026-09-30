#ifndef WHEEL_BUTTONS_H_
#define WHEEL_BUTTONS_H_

#include <stdbool.h>
#include <stdint.h>

struct wheel_button_state {
	bool held;
	bool releasing;
	int64_t release_since_ms;
};

struct wheel_buttons {
	struct wheel_button_state buttons[3];
	bool sampled;
	int64_t last_sample_ms;
};

/* Zero-initialize state. Accept presses immediately, but require an observed
 * stable release before accepting another press. Call on valid frames only. */
uint8_t wheel_buttons_update(struct wheel_buttons *state, uint8_t raw, int64_t now_ms);

#endif
