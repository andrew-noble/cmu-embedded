#include <zephyr/kernel.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>

#include "blinker.h"
#include "config.h"
#include "crc8_checksum.h"
#include "motor_control.h"
#include "servo_driver.h"
#include "status_tx.h"
#include "wheel_buttons.h"

#define UART_DEVICE_NODE DT_NODELABEL(usart1)

/* One-slot queue: the newest complete command frame from the UART ISR. */
struct received_frame {
	int64_t received_ms;
	uint8_t data[FRAME_LEN];
};
K_MSGQ_DEFINE(frame_queue, sizeof(struct received_frame), UART_RX_QUEUE_DEPTH,
	      UART_RX_QUEUE_ALIGNMENT);
/* Serialize command outputs with fail-safe so timeout cannot be undone halfway
 * through applying a command. Motor priority 0 can still preempt both paths. */
K_MUTEX_DEFINE(command_lock);
static int64_t link_deadline_ms;
static uint8_t link_miss_count; /* 0..LINK_MISS_LIMIT; guarded by command_lock */
BUILD_ASSERT(LINK_MISS_INTERVAL_MS > 0);
BUILD_ASSERT(LINK_MISS_LIMIT > 0 && LINK_MISS_LIMIT <= UINT8_MAX);

/* Count elapsed intervals, not semaphore wakes: wakes may coalesce or belong
 * to an earlier command. Clamp before narrowing so the counter never wraps. */
static uint8_t link_misses_at(int64_t now_ms, int64_t deadline_ms)
{
	int64_t elapsed = now_ms - (deadline_ms - LINK_TIMEOUT_MS);
	return (uint8_t)CLAMP(elapsed / LINK_MISS_INTERVAL_MS, 0, LINK_MISS_LIMIT);
}
static const struct device *const uart_device = DEVICE_DT_GET(UART_DEVICE_NODE);

/* True while valid commands are arriving. Set by cmd_handler; cleared by
 * enter_error_state() from cmd_handler or fail_safe. */
static atomic_t link_ok = ATOMIC_INIT(0);   /* power up in the error state */

/* Decoded command from the Pi. */
struct pi_command {
	int16_t steer;
	int16_t throttle;
	int16_t brake;
	uint8_t buttons;
	uint8_t seq;
};

enum frame_result {
	FRAME_OK,
	FRAME_BAD_CRC,
	FRAME_OUT_OF_RANGE,
};

#define CHECK_INPUT_CLAMP(name) \
	BUILD_ASSERT(name##_CLAMP_MIN >= name##_RAW_MIN && \
		     name##_CLAMP_MAX <= name##_RAW_MAX && \
		     name##_CLAMP_MIN <= name##_CLAMP_MAX)
CHECK_INPUT_CLAMP(STEER);
CHECK_INPUT_CLAMP(THROTTLE);
CHECK_INPUT_CLAMP(BRAKE);
BUILD_ASSERT(THROTTLE_CLAMP_MIN <= THROTTLE_RAW_REST &&
	     THROTTLE_CLAMP_MAX >= THROTTLE_RAW_REST,
	     "Throttle clamp must include released pedal; otherwise release commands drive");
BUILD_ASSERT(INPUT_CLAMP_REPORT_MS > 0);

static uint8_t clamp_inputs(struct pi_command *command)
{
	struct pi_command raw = *command;
	command->steer = CLAMP(raw.steer, STEER_CLAMP_MIN, STEER_CLAMP_MAX);
	command->throttle = CLAMP(raw.throttle, THROTTLE_CLAMP_MIN, THROTTLE_CLAMP_MAX);
	command->brake = CLAMP(raw.brake, BRAKE_CLAMP_MIN, BRAKE_CLAMP_MAX);
	return (raw.steer != command->steer ? 1 : 0) |
	       (raw.throttle != command->throttle ? 2 : 0) |
	       (raw.brake != command->brake ? 4 : 0);
}

static void report_input_clamps(const struct pi_command *raw,
				const struct pi_command *applied, uint8_t mask)
{
	if (mask & 1) {
		printk("CLAMP: steering out of range: received=%d applied=%d allowed=[%d,%d]\n",
		       raw->steer, applied->steer, STEER_CLAMP_MIN, STEER_CLAMP_MAX);
	}
	if (mask & 2) {
		printk("CLAMP: throttle out of range: received=%d applied=%d allowed=[%d,%d]\n",
		       raw->throttle, applied->throttle, THROTTLE_CLAMP_MIN, THROTTLE_CLAMP_MAX);
	}
	if (mask & 4) {
		printk("CLAMP: brake out of range: received=%d applied=%d allowed=[%d,%d]\n",
		       raw->brake, applied->brake, BRAKE_CLAMP_MIN, BRAKE_CLAMP_MAX);
	}
}

/* Byte positions after the 4-byte header (SOF, ID_HI, ID_LO, DLC). */
enum {
	POS_STEER = 4,
	POS_THROTTLE = 6,
	POS_BRAKE = 8,
	POS_BUTTONS = 10,
	POS_SEQ = 11,
	POS_CRC = 12,
};

/* ---------------------------------------------------------------------------
 * Link watchdog (step 2)
 *
 * link_timer checks each missed interval from the reception timestamp of
 * each valid command. LINK_MISS_LIMIT consecutive misses trigger fail-safe.
 * The expiry function runs in interrupt
 * context, so it only gives a semaphore; fail_safe does the work.
 * ------------------------------------------------------------------------- */
K_SEM_DEFINE(failsafe_sem, 0, 1);

static void link_timer_expiry(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	k_sem_give(&failsafe_sem);
}

K_TIMER_DEFINE(link_timer, link_timer_expiry, NULL);

/* Hazard-state outputs: the motor thread brakes on an invalid command,
 * steering PWM stops, hazards flash, and the status frame reports the error. */
static void apply_hazard_outputs(void)
{
	motor_control_submit((struct motor_command){0});   /* valid = false: brake */
	if (servo_driver_disable() != 0) {
		printk("ERROR: could not disable steering PWM\n");
	}
	blinker_set_mode(BLINKER_HAZARD);
	status_tx_set_zone_state(ZONE_STATE_ERROR);
}

/* Fail-safe: hazard outputs and link marked down. Returns true if the system
 * was in normal operation before this call. */
static bool enter_error_state(void)
{
	bool was_ok = atomic_clear(&link_ok) != 0;
	k_timer_stop(&link_timer);

	apply_hazard_outputs();
	return was_ok;
}

/* ---------------------------------------------------------------------------
 * Wheel buttons (Pi byte 10). Used only by cmd_handler.
 * ------------------------------------------------------------------------- */
static bool self_test;
static int64_t self_test_press_ms;   /* last press while in self-test; -1 = none */

/* A press enters self-test. Once in, two presses within
 * SELF_TEST_EXIT_WINDOW_MS exit; the entering press does not count. */
static void update_self_test(bool pressed)
{
	if (!pressed) {
		return;
	}

	int64_t now = k_uptime_get();

	if (!self_test) {
		self_test = true;
		self_test_press_ms = -1;
		apply_hazard_outputs();
		printk("SELF-TEST: hazards on, dynamic braking\n");
	} else if (self_test_press_ms >= 0 &&
		   now - self_test_press_ms <= SELF_TEST_EXIT_WINDOW_MS) {
		self_test = false;
		printk("SELF-TEST exit\n");
	} else {
		self_test_press_ms = now;
	}
}

/* UART ISR callback: assemble 13-byte command frames and push each complete
 * frame to frame_queue (newest wins). Header checks only; the CRC and value
 * checks run in cmd_handler. */
static void get_uart_frame(const struct device *device, void *user_data) {
	ARG_UNUSED(user_data);

	static uint8_t frame[FRAME_LEN];
	uint8_t c;
	static int n = 0;   /* bytes stored in the current frame */

	// start processing interrupt
	uart_irq_update(device);
	// Check if there are bytes in the receiving buffer
	if (uart_irq_rx_ready(device) <= 0) {
		return;
	}

	// Move bytes from buffer to frame
	while (uart_fifo_read(device, &c, 1) == 1) {
		if (n == 0 && c != UART_SOF) continue;
		frame[n++] = c;
		/* n was just incremented, so n == 2 checks frame[1], and so on. */
		if ((n == 2 && c != CMD_ID_HI) || (n == 3 && c != CMD_ID_LO) ||
		    (n == 4 && c != CMD_DLC)) {
			n = (c == UART_SOF) ? (frame[0] = c, 1) : 0;
			continue;
		}
		if (n == FRAME_LEN) {
			/* Keep only the newest frame: drop one cmd_handler has not taken. */
			if (k_msgq_num_free_get(&frame_queue) == 0) {
				struct received_frame discard;
				k_msgq_get(&frame_queue, &discard, K_NO_WAIT);
			}
			/* Cannot fail: the slot is free and no thread runs inside an ISR. */
			struct received_frame received = { .received_ms = k_uptime_get() };
			memcpy(received.data, frame, FRAME_LEN);
			(void)k_msgq_put(&frame_queue, &received, K_NO_WAIT);
			n = 0;
		}
	}
}

/* Helper fxn for cmd_handler thread. The ISR already checked the header. */
static enum frame_result parse_frame(const uint8_t f[FRAME_LEN], struct pi_command *out) {
	/* CRC covers bytes 1 to 11 (ID through SEQ), same as the Pi. */
	if (crc8(&f[1], FRAME_LEN - 2) != f[POS_CRC]) {
		return FRAME_BAD_CRC;
	}

	out->steer    = (int16_t)sys_get_le16(&f[POS_STEER]);
	out->throttle = (int16_t)sys_get_le16(&f[POS_THROTTLE]);
	out->brake    = (int16_t)sys_get_le16(&f[POS_BRAKE]);
	out->buttons  = f[POS_BUTTONS];
	out->seq      = f[POS_SEQ];

	if ((out->steer < STEER_RAW_MIN || out->steer > STEER_RAW_MAX) ||
		(out->throttle < THROTTLE_RAW_MIN || out->throttle > THROTTLE_RAW_MAX) ||
	    (out->brake < BRAKE_RAW_MIN || out->brake > BRAKE_RAW_MAX)) {
		return FRAME_OUT_OF_RANGE;
	}

	return FRAME_OK;
}

/* cmd_handler: one run per frame from the Pi (sporadic, deadline 2 ms). */
static void cmd_handler_entry(void *p1, void *p2, void *p3) {
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	int64_t last_print_ms = 0;
	int64_t last_clamp_report_ms = 0;
	uint8_t previous_clamp_mask = 0;
	uint32_t n_valid = 0, n_bad = 0;
	struct received_frame received;
	struct wheel_buttons buttons = {0};
	struct pi_command pi_cmd;

	while (1) {
		/* Sleep until the ISR delivers a frame. Link loss is detected by
		 * link_timer, so no timeout is needed here. */
		k_msgq_get(&frame_queue, &received, K_FOREVER);

		enum frame_result r = parse_frame(received.data, &pi_cmd);

		if (r == FRAME_BAD_CRC) {
			/* Corrupted on the wire: drop and count. A dead link is
			 * caught by link_timer, not by one bad frame. */
			n_bad++;
			continue;
		}
		/* Reject protocol-invalid values before any operational clamping. */
		struct pi_command raw_command = pi_cmd;
		uint8_t clamp_mask = r == FRAME_OK ? clamp_inputs(&pi_cmd) : 0;

		k_mutex_lock(&command_lock, K_FOREVER);
		int64_t deadline_ms = received.received_ms + LINK_TIMEOUT_MS;
		if (k_uptime_get() >= deadline_ms) {
			/* An old queued frame must never restart the watchdog or outputs. */
			n_bad++;
			k_timer_stop(&link_timer);
			enter_error_state();
			k_mutex_unlock(&command_lock);
			continue;
		}
		if (!atomic_get(&link_ok)) {
			/* Do not combine self-test exit clicks across a lost link. */
			self_test_press_ms = -1;
		}

		/* Debounced press edges for all three wheel buttons. */
		uint8_t pressed = 0;
		bool exited_self_test = false;

		if (r == FRAME_OK) {
			bool was_self_test = self_test;

			pressed = wheel_buttons_update(&buttons, pi_cmd.buttons, received.received_ms);
			update_self_test((pressed & BUTTON_SELF_TEST) != 0);
			exited_self_test = was_self_test && !self_test;
		}

		struct motor_command command = {
			.throttle = pi_cmd.throttle,
			.brake = pi_cmd.brake,
			.clutch = 0, /* byte 10 is buttons, not clutch */
			.valid = true,
		};

		/* Out-of-range values mean error state (R3). Short-circuit: a
		 * command that failed parse_frame is never submitted. In self-test
		 * the hazard outputs stay applied, so drive and steering are skipped. */
		if (r == FRAME_OUT_OF_RANGE ||
		    (!self_test && (!motor_control_submit(command) ||
				    servo_driver_set_steering(pi_cmd.steer) != 0))) {
			n_bad++;
			k_timer_stop(&link_timer);   /* already failed; no LINK LOST later */
			if (enter_error_state()) {
				printk("ERROR: invalid command or steering output failure, fail-safe\n");
			}
			k_mutex_unlock(&command_lock);
			continue;
		}

		/* Processing time must not extend the receive-to-fail-safe deadline. */
		int64_t now_ms = k_uptime_get();
		int64_t remaining_ms = deadline_ms - now_ms;
		if (remaining_ms <= 0) {
			k_timer_stop(&link_timer);
			enter_error_state();
			k_mutex_unlock(&command_lock);
			continue;
		}
		link_deadline_ms = deadline_ms;
		/* A fresh command resets the counter. Account for any intervals that
		 * elapsed during processing and align the next tick to reception. */
		link_miss_count = link_misses_at(now_ms, deadline_ms);
		int64_t next_tick_ms = LINK_MISS_INTERVAL_MS -
			(now_ms - received.received_ms) % LINK_MISS_INTERVAL_MS;
		k_timer_start(&link_timer, K_MSEC(next_tick_ms), K_MSEC(LINK_MISS_INTERVAL_MS));
		n_valid++;

		bool link_came_up = !atomic_set(&link_ok, 1);

		if (link_came_up) {
			printk("LINK UP\n");
		}
		if (!self_test) {
			if (link_came_up || exited_self_test) {
				/* Leaving the hazard state: start with no turn signal. */
				blinker_set_mode(BLINKER_OFF);
				status_tx_set_zone_state(ZONE_STATE_NORMAL);
			}
			blinker_update_turn(pressed, pi_cmd.steer);
		}
		k_mutex_unlock(&command_lock);

		/* Report after applying outputs and outside the command mutex. */
		int64_t report_ms = k_uptime_get();
		uint8_t cleared_mask = previous_clamp_mask & ~clamp_mask;
		if (cleared_mask & 1) {
			printk("CLAMP: steering back in range; clamp cleared\n");
		}
		if (cleared_mask & 2) {
			printk("CLAMP: throttle back in range; clamp cleared\n");
		}
		if (cleared_mask & 4) {
			printk("CLAMP: brake back in range; clamp cleared\n");
		}
		if (clamp_mask && (clamp_mask != previous_clamp_mask ||
		    report_ms - last_clamp_report_ms >= INPUT_CLAMP_REPORT_MS)) {
			report_input_clamps(&raw_command, &pi_cmd, clamp_mask);
			last_clamp_report_ms = report_ms;
		}
		previous_clamp_mask = clamp_mask;

		if (n_valid % PRINT_EVERY == 0) {
			printk("steer %6d  thr %6d  brk %6d  buttons %3u  seq %3u  bad %u\n",
			       raw_command.steer, raw_command.throttle, raw_command.brake,
			       (unsigned)pi_cmd.buttons, (unsigned)pi_cmd.seq,
			       (unsigned)n_bad);
		}

		if (k_uptime_get() - last_print_ms >= MAIN_PRINT_PERIOD_MS) {
			struct motor_telemetry telemetry;

			motor_control_get_telemetry(&telemetry);
			last_print_ms = k_uptime_get();
			printk("ENC L %8d %5d rpm | R %8d %5d rpm | target %3d duty %3d%% %s"
			       " | pwm L=%d R=%d\n",
			       (int)telemetry.left_position, (int)telemetry.left_rpm,
			       (int)telemetry.right_position, (int)telemetry.right_rpm,
			       (int)telemetry.target_rpm, (int)telemetry.duty_percent,
			       telemetry.reverse ? "REV" : "FWD",
			       (int)telemetry.left_duty_percent, (int)telemetry.right_duty_percent);
		}
	}
}

/* fail_safe: reception timeout configured by LINK_TIMEOUT_MS. */
static void fail_safe_entry(void *p1, void *p2, void *p3) {
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		k_sem_take(&failsafe_sem, K_FOREVER);
		k_mutex_lock(&command_lock, K_FOREVER);
		/* A newer command may have renewed the deadline after the timer fired. */
		bool lost = false;
		if (atomic_get(&link_ok)) {
			link_miss_count = link_misses_at(k_uptime_get(), link_deadline_ms);
			lost = link_miss_count == LINK_MISS_LIMIT;
		}
		if (lost) {
			enter_error_state();
		}
		k_mutex_unlock(&command_lock);
		if (lost) {
			printk("LINK LOST (%d consecutive %d ms misses; %d ms total)\n",
			       LINK_MISS_LIMIT, LINK_MISS_INTERVAL_MS, LINK_TIMEOUT_MS);
		}
	}
}

K_THREAD_DEFINE(fail_safe_tid, FAIL_SAFE_STACK_SIZE, fail_safe_entry,
		NULL, NULL, NULL, FAIL_SAFE_PRIORITY, 0, 0);
K_THREAD_DEFINE(cmd_handler_tid, CMD_HANDLER_STACK_SIZE, cmd_handler_entry,
		NULL, NULL, NULL, CMD_HANDLER_PRIORITY, 0, CMD_HANDLER_START_DELAY_MS);

int main(void) {
	/* Power-up is an error state until a valid Pi command arrives. */
	blinker_set_mode(BLINKER_HAZARD);

	if (servo_driver_init() != 0) {
		printk("Steering PWM not ready, check overlay\n");
		return 0;
	}

	if (!device_is_ready(uart_device)) {
		printk("USART1 not ready, check overlay\n");
		return 0;
	}

	int ret = uart_irq_callback_user_data_set(uart_device, get_uart_frame, NULL);

	if (ret < 0) {
		printk("Error setting UART callback: %d\n", ret);
		return 0;
	}

	uart_irq_rx_enable(uart_device);
	printk("CRC self-test: 0x%02x (expect 0xf4)\n", crc8((const uint8_t *)"123456789", 9));
	printk("Waiting for commands (link DOWN)\n");

	return 0;
}
