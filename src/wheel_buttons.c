#include "wheel_buttons.h"
#include "config.h"

uint8_t wheel_buttons_update(struct wheel_buttons *state, uint8_t raw, int64_t now_ms)
{
	const uint8_t masks[] = { BUTTON_SELF_TEST, BUTTON_LEFT, BUTTON_RIGHT };
	uint8_t pressed = 0;
	bool gap = state->sampled && now_ms - state->last_sample_ms >= LINK_TIMEOUT_MS;
	state->sampled = true;
	state->last_sample_ms = now_ms;

	for (unsigned i = 0; i < 3; i++) {
		struct wheel_button_state *button = &state->buttons[i];
		if (gap) {
			/* Silence is not evidence of a stable release. */
			button->releasing = false;
		}
		if (raw & masks[i]) {
			/* A new press can itself finish the observed release interval.
			 * Requiring a second released packet loses fast double clicks at
			 * the Pi's 50 ms sampling interval. A long link gap cannot rearm. */
			if (button->releasing &&
			    now_ms - button->release_since_ms >= BUTTON_DEBOUNCE_MS) {
				button->held = false;
			}
			button->releasing = false;
			if (!button->held) {
				button->held = true;
				pressed |= masks[i];
			}
		} else if (button->held) {
			if (!button->releasing) {
				button->releasing = true;
				button->release_since_ms = now_ms;
			} else if (now_ms - button->release_since_ms >= BUTTON_DEBOUNCE_MS) {
				button->held = false;
				button->releasing = false;
			}
		}
	}
	return pressed;
}
