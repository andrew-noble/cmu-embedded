#include <zephyr/kernel.h>
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

#define UART_DEVICE_NODE DT_NODELABEL(usart1)

/* One-slot queue: the newest complete command frame from the UART ISR. */
K_MSGQ_DEFINE(frame_queue, FRAME_LEN, UART_RX_QUEUE_DEPTH, UART_RX_QUEUE_ALIGNMENT);
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
 * link_timer is a one-shot timer restarted by cmd_handler on every valid
 * command. If LINK_TIMEOUT_MS passes without one (three missed updates),
 * it expires and wakes fail_safe. The expiry function runs in interrupt
 * context, so it only gives a semaphore; fail_safe does the work.
 * ------------------------------------------------------------------------- */
K_SEM_DEFINE(failsafe_sem, 0, 1);

static void link_timer_expiry(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	k_sem_give(&failsafe_sem);
}

K_TIMER_DEFINE(link_timer, link_timer_expiry, NULL);

/* Fail-safe outputs: the motor thread brakes on an invalid command, hazards
 * flash, and the status frame reports the error. Returns true if the system
 * was in normal operation before this call. */
static bool enter_error_state(void)
{
	bool was_ok = atomic_clear(&link_ok) != 0;

	motor_control_submit((struct motor_command){0});   /* valid = false: brake */
	if (servo_driver_disable() != 0) {
		printk("ERROR: could not disable steering PWM\n");
	}
	blinker_set_mode(BLINKER_HAZARD);
	status_tx_set_zone_state(ZONE_STATE_ERROR);
	return was_ok;
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
				uint8_t discard[FRAME_LEN];
				k_msgq_get(&frame_queue, discard, K_NO_WAIT);
			}
			/* Cannot fail: the slot is free and no thread runs inside an ISR. */
			(void)k_msgq_put(&frame_queue, frame, K_NO_WAIT);
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
	uint32_t n_valid = 0, n_bad = 0;
	uint8_t buf[FRAME_LEN];
	struct pi_command pi_cmd;

	while (1) {
		/* Sleep until the ISR delivers a frame. Link loss is detected by
		 * link_timer, so no timeout is needed here. */
		k_msgq_get(&frame_queue, buf, K_FOREVER);

		enum frame_result r = parse_frame(buf, &pi_cmd);

		if (r == FRAME_BAD_CRC) {
			/* Corrupted on the wire: drop and count. A dead link is
			 * caught by link_timer, not by one bad frame. */
			n_bad++;
			continue;
		}

		struct motor_command command = {
			.throttle = pi_cmd.throttle,
			.brake = pi_cmd.brake,
			.clutch = 0, /* Pi bridge.c sends buttons placeholder, not clutch. */
			.valid = true,
		};

		/* Out-of-range values mean error state (R3). Short-circuit: a
		 * command that failed parse_frame is never submitted. */
		if (r == FRAME_OUT_OF_RANGE || !motor_control_submit(command) ||
		    servo_driver_set_steering(pi_cmd.steer) != 0) {
			n_bad++;
			k_timer_stop(&link_timer);   /* already failed; no LINK LOST later */
			if (enter_error_state()) {
				printk("ERROR: invalid command or steering output failure, fail-safe\n");
			}
			continue;
		}

		/* Valid command: feed the link watchdog. */
		k_timer_start(&link_timer, K_MSEC(LINK_TIMEOUT_MS), K_NO_WAIT);
		n_valid++;

		if (!atomic_set(&link_ok, 1)) {
			/* First valid command after power-up or an error. */
			blinker_set_mode(BLINKER_OFF);
			status_tx_set_zone_state(ZONE_STATE_NORMAL);
			printk("LINK UP\n");
		}

		if (n_valid % PRINT_EVERY == 0) {
			printk("steer %6d  thr %6d  brk %6d  buttons %3u  seq %3u  bad %u\n",
			       pi_cmd.steer, pi_cmd.throttle, pi_cmd.brake,
			       (unsigned)pi_cmd.buttons, (unsigned)pi_cmd.seq,
			       (unsigned)n_bad);
		}

		if (k_uptime_get() - last_print_ms >= MAIN_PRINT_PERIOD_MS) {
			struct motor_telemetry telemetry;

			motor_control_get_telemetry(&telemetry);
			last_print_ms = k_uptime_get();
			printk("ENC L %8d %5d rpm | R %8d %5d rpm | target %3d duty %3d%% %s\n",
			       (int)telemetry.left_position, (int)telemetry.left_rpm,
			       (int)telemetry.right_position, (int)telemetry.right_rpm,
			       (int)telemetry.target_rpm, (int)telemetry.duty_percent,
			       telemetry.reverse ? "REV" : "FWD");
		}
	}
}

/* fail_safe: runs only when link_timer expires (sporadic, at least 150 ms
 * apart). Must finish within 100 ms of the expiry; takes microseconds. */
static void fail_safe_entry(void *p1, void *p2, void *p3) {
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		k_sem_take(&failsafe_sem, K_FOREVER);
		if (enter_error_state()) {
			printk("LINK LOST (no valid command for %d ms)\n", LINK_TIMEOUT_MS);
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
