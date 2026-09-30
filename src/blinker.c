#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

#include "blinker.h"
#include "config.h"

#define BLINKER_NODE DT_PATH(zephyr_user)

/* Fail the build, rather than run without LEDs, if a property is missing or
 * misspelled in the board overlay. */
BUILD_ASSERT(DT_NODE_HAS_PROP(BLINKER_NODE, blinker_left_front_gpios) &&
	     DT_NODE_HAS_PROP(BLINKER_NODE, blinker_left_rear_gpios) &&
	     DT_NODE_HAS_PROP(BLINKER_NODE, blinker_right_front_gpios) &&
	     DT_NODE_HAS_PROP(BLINKER_NODE, blinker_right_rear_gpios),
	     "Define all four blinker-*-gpios properties in the board overlay");

static const struct gpio_dt_spec left_front =
	GPIO_DT_SPEC_GET(BLINKER_NODE, blinker_left_front_gpios);
static const struct gpio_dt_spec left_rear =
	GPIO_DT_SPEC_GET(BLINKER_NODE, blinker_left_rear_gpios);
static const struct gpio_dt_spec right_front =
	GPIO_DT_SPEC_GET(BLINKER_NODE, blinker_right_front_gpios);
static const struct gpio_dt_spec right_rear =
	GPIO_DT_SPEC_GET(BLINKER_NODE, blinker_right_rear_gpios);

static struct k_spinlock state_lock;
static enum blinker_mode requested_mode;
static uint32_t generation;
static bool phase_on;
static bool turn_seen;

enum blinker_mode blinker_get_mode(void)
{
	k_spinlock_key_t key = k_spin_lock(&state_lock);
	enum blinker_mode mode = requested_mode;
	k_spin_unlock(&state_lock, key);
	return mode;
}

BUILD_ASSERT(BLINKER_TURN_THRESHOLD > STEER_DEAD_ZONE &&
	     BLINKER_TURN_THRESHOLD <= STEER_RAW_MAX &&
	     BLINKER_TURN_THRESHOLD <= -STEER_RAW_MIN);
K_SEM_DEFINE(blinker_wake, 0, 1);

static void blinker_tick(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	k_sem_give(&blinker_wake);
}

K_TIMER_DEFINE(blinker_timer, blinker_tick, NULL);

/* Caller holds state_lock so fail-safe hazards cannot be overwritten by
 * a cancellation based on an older mode. Timer operations do not wait. */
static void set_mode_locked(enum blinker_mode mode)
{
	if (requested_mode == mode) {
		return;
	}
	requested_mode = mode;
	turn_seen = false;
	generation++;
	phase_on = mode != BLINKER_OFF;
	k_timer_stop(&blinker_timer);
	if (mode != BLINKER_OFF) {
		int32_t half_period = mode == BLINKER_HAZARD ?
			HAZARD_HALF_PERIOD_MS : TURN_HALF_PERIOD_MS;
		k_timer_start(&blinker_timer, K_MSEC(half_period), K_MSEC(half_period));
	}
	k_sem_give(&blinker_wake);
}

void blinker_set_mode(enum blinker_mode mode)
{
	if (mode < BLINKER_OFF || mode > BLINKER_HAZARD) {
		return;
	}
	k_spinlock_key_t key = k_spin_lock(&state_lock);
	set_mode_locked(mode);
	k_spin_unlock(&state_lock, key);
}

void blinker_update_turn(uint8_t pressed, int16_t steering)
{
	k_spinlock_key_t key = k_spin_lock(&state_lock);

	/* Hazards belong to self-test/fail-safe, never to steering cancellation. */
	if (requested_mode == BLINKER_HAZARD) {
		k_spin_unlock(&state_lock, key);
		return;
	}
	switch (pressed & (BUTTON_LEFT | BUTTON_RIGHT)) {
	case BUTTON_LEFT:
		set_mode_locked(requested_mode == BLINKER_LEFT ? BLINKER_OFF : BLINKER_LEFT);
		break;
	case BUTTON_RIGHT:
		set_mode_locked(requested_mode == BLINKER_RIGHT ? BLINKER_OFF : BLINKER_RIGHT);
		break;
	default:
		break; /* simultaneous presses do not change the selected side */
	}

	/* Negative input is left. First observe a turn in the selected direction,
	 * then cancel upon reaching or crossing neutral (including skipped samples). */
	if (requested_mode == BLINKER_LEFT) {
		if (steering <= -BLINKER_TURN_THRESHOLD) {
			turn_seen = true;
		} else if (turn_seen && steering >= -STEER_DEAD_ZONE) {
			set_mode_locked(BLINKER_OFF);
		}
	} else if (requested_mode == BLINKER_RIGHT) {
		if (steering >= BLINKER_TURN_THRESHOLD) {
			turn_seen = true;
		} else if (turn_seen && steering <= STEER_DEAD_ZONE) {
			set_mode_locked(BLINKER_OFF);
		}
	}
	k_spin_unlock(&state_lock, key);
}

static int configure_pins(void)
{
	const struct gpio_dt_spec *pins[] = {
		&left_front, &left_rear, &right_front, &right_rear
	};

	for (size_t i = 0; i < ARRAY_SIZE(pins); i++) {
		if (!gpio_is_ready_dt(pins[i]) ||
		    gpio_pin_configure_dt(pins[i], GPIO_OUTPUT_INACTIVE) != 0) {
			return -ENODEV;
		}
	}
	return 0;
}

static int write_pins(enum blinker_mode mode, bool on)
{
	bool left = on && (mode == BLINKER_LEFT || mode == BLINKER_HAZARD);
	bool right = on && (mode == BLINKER_RIGHT || mode == BLINKER_HAZARD);
	int rc = gpio_pin_set_dt(&left_front, left);

	rc |= gpio_pin_set_dt(&left_rear, left);
	rc |= gpio_pin_set_dt(&right_front, right);
	rc |= gpio_pin_set_dt(&right_rear, right);
	return rc;
}

static void blinker_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	if (configure_pins() != 0) {
		printk("Blinker GPIO setup failed\n");
		return;
	}

	uint32_t seen_generation = UINT32_MAX;

	while (1) {
		k_sem_take(&blinker_wake, K_FOREVER);
		uint32_t ticks = k_timer_status_get(&blinker_timer);
		k_spinlock_key_t key = k_spin_lock(&state_lock);
		enum blinker_mode mode = requested_mode;

		if (seen_generation != generation) {
			seen_generation = generation;
		} else if (mode != BLINKER_OFF && (ticks & 1U) != 0) {
			phase_on = !phase_on;
		}
		bool on = phase_on;
		k_spin_unlock(&state_lock, key);

		if (write_pins(mode, on) != 0) {
			printk("Blinker GPIO write failed\n");
			return;
		}
	}
}

/* Lower priority than the motor thread; no delay loop or GPIO work in an ISR. */
K_THREAD_DEFINE(blinker_tid, BLINKER_STACK_SIZE, blinker_thread,
		NULL, NULL, NULL, BLINKER_PRIORITY, 0, 0);
