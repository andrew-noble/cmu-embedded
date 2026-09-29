#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>

#include <stdlib.h>
#include <string.h>

#include "motor_calibration_config.h"
#include "config.h"
#include "encoder.h"
#include "motor_driver.h"

struct calibration_telemetry {
	int32_t left_position;
	int32_t right_position;
	int32_t left_rpm;
	int32_t right_rpm;
	int32_t duty_percent;
	bool calibrating;
};

static struct k_spinlock calibration_lock;
static struct calibration_telemetry latest_telemetry;
static bool calibration_ready;
static bool calibration_active;
static uint16_t calibration_duty;
static int64_t calibration_deadline_ms;
K_SEM_DEFINE(calibration_wake, 0, 1);

BUILD_ASSERT(CALIBRATION_MAX_DUTY_PERCENT > 0 &&
	     CALIBRATION_MAX_DUTY_PERCENT <= 100);

static bool motor_calibration_arm(void)
{
	k_spinlock_key_t key = k_spin_lock(&calibration_lock);
	bool ready = calibration_ready && CALIBRATION_OUTPUTS_ENABLED;

	if (ready) {
		calibration_active = true;
		calibration_duty = 0;
		calibration_deadline_ms = k_uptime_get() + CAL_COMMAND_TIMEOUT_MS;
	}
	k_spin_unlock(&calibration_lock, key);
	if (ready) {
		k_sem_give(&calibration_wake);
	}
	return ready;
}

static bool motor_calibration_set_duty(uint16_t duty_percent)
{
	if (duty_percent > CALIBRATION_MAX_DUTY_PERCENT) {
		return false;
	}
	k_spinlock_key_t key = k_spin_lock(&calibration_lock);
	bool active = calibration_active && calibration_ready;

	if (active) {
		calibration_duty = duty_percent;
		calibration_deadline_ms = k_uptime_get() + CAL_COMMAND_TIMEOUT_MS;
	}
	k_spin_unlock(&calibration_lock, key);
	if (active) {
		k_sem_give(&calibration_wake);
	}
	return active;
}

static void motor_calibration_stop(void)
{
	k_spinlock_key_t key = k_spin_lock(&calibration_lock);

	calibration_active = false;
	calibration_duty = 0;
	k_spin_unlock(&calibration_lock, key);
	k_sem_give(&calibration_wake);
}

static uint16_t motor_calibration_max_duty(void)
{
	return CALIBRATION_MAX_DUTY_PERCENT;
}

static void motor_calibration_get_telemetry(struct calibration_telemetry *telemetry)
{
	k_spinlock_key_t key = k_spin_lock(&calibration_lock);

	*telemetry = latest_telemetry;
	k_spin_unlock(&calibration_lock, key);
}

static void control_tick(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	k_sem_give(&calibration_wake);
}

K_TIMER_DEFINE(calibration_timer, control_tick, NULL);

static void calibration_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	struct encoder_sample sample = { 0 };
	int64_t last_sample_ms = k_uptime_get();
	bool fault = false;

	if (!motor_driver_init() || encoder_init() != 0) {
		printk("CAL_ERROR,hardware_not_ready\n");
		return;
	}

	k_spinlock_key_t key = k_spin_lock(&calibration_lock);
	calibration_ready = true;
	k_spin_unlock(&calibration_lock, key);
	k_timer_start(&calibration_timer, K_MSEC(ENC_PERIOD_MS),
		      K_MSEC(ENC_PERIOD_MS));
	printk("CAL_READY,max_duty=%u\n",
	       (unsigned)CALIBRATION_MAX_DUTY_PERCENT);

	while (1) {
		k_sem_take(&calibration_wake, K_FOREVER);
		int64_t now_ms = k_uptime_get();

		key = k_spin_lock(&calibration_lock);
		bool active = calibration_active;
		uint16_t duty = calibration_duty;
		if (active && now_ms >= calibration_deadline_ms) {
			calibration_active = false;
			calibration_duty = 0;
			active = false;
			duty = 0;
		}
		k_spin_unlock(&calibration_lock, key);

		if (k_timer_status_get(&calibration_timer) > 0) {
			int32_t elapsed_ms = (int32_t)(now_ms - last_sample_ms);

			last_sample_ms = now_ms;
			if (elapsed_ms <= 0 || encoder_sample(elapsed_ms, &sample) != 0) {
				fault = true;
			}
		}

		if (fault) {
			key = k_spin_lock(&calibration_lock);
			calibration_ready = false;
			calibration_active = false;
			calibration_duty = 0;
			k_spin_unlock(&calibration_lock, key);
			active = false;
			duty = 0;
		}

		if (active && duty > 0) {
			if (motor_driver_drive(false, duty) != 0) {
				fault = true;
				motor_driver_brake();
			}
		} else if (motor_driver_brake() != 0) {
			fault = true;
		}

		key = k_spin_lock(&calibration_lock);
		if (fault) {
			calibration_ready = false;
			calibration_active = false;
			calibration_duty = 0;
		}
		latest_telemetry = (struct calibration_telemetry) {
			.left_position = sample.left_position,
			.right_position = sample.right_position,
			.left_rpm = sample.left_rpm,
			.right_rpm = sample.right_rpm,
			.duty_percent = active && !fault ? duty : 0,
			.calibrating = active && !fault,
		};
		k_spin_unlock(&calibration_lock, key);
	}
}

K_THREAD_DEFINE(calibration_tid, CAL_MOTOR_STACK_SIZE, calibration_thread,
		NULL, NULL, NULL, CAL_MOTOR_PRIORITY, 0, 0);

/* This firmware is built separately from the Pi-controlled application.
 * Only exact console commands can arm or change the open-loop duty. */
#define CONSOLE_NODE DT_CHOSEN(zephyr_console)
static const struct device *const console = DEVICE_DT_GET(CONSOLE_NODE);

static void handle_line(const char *line)
{
	if (strcmp(line, "CAL ARM") == 0) {
		if (motor_calibration_arm()) {
			printk("CAL_ARMED,max_duty=%u\n",
			       (unsigned)motor_calibration_max_duty());
		} else {
			printk("CAL_ERROR,not_ready\n");
		}
		return;
	}

	if (strcmp(line, "CAL STOP") == 0) {
		motor_calibration_stop();
		printk("CAL_STOPPED\n");
		return;
	}

	if (strncmp(line, "CAL DUTY ", 9) == 0) {
		char *end;
		unsigned long value = strtoul(line + 9, &end, 10);

		if (end == line + 9 || *end != '\0' || value > UINT16_MAX ||
		    !motor_calibration_set_duty((uint16_t)value)) {
			printk("CAL_ERROR,bad_duty_or_timeout\n");
		} else {
			printk("CAL_DUTY,%u\n", (unsigned)value);
		}
		return;
	}

	printk("CAL_ERROR,unknown_command\n");
}

int main(void)
{
	char line[CAL_INPUT_LINE_BYTES];
	size_t length = 0;
	bool discard_line = false;
	int64_t last_report_ms = 0;

	if (!device_is_ready(console)) {
		return 0;
	}
	printk("CAL_CONSOLE_READY: CAL ARM, CAL DUTY <0..max>, CAL STOP\n");

	while (1) {
		unsigned char c;

		while (uart_poll_in(console, &c) == 0) {
			if (c == '\r') {
				continue;
			}
			if (c == '\n') {
				line[length] = '\0';
				if (!discard_line && length > 0) {
					handle_line(line);
				}
				length = 0;
				discard_line = false;
			} else if (discard_line) {
				continue;
			} else if (length < sizeof(line) - 1) {
				line[length++] = (char)c;
			} else {
				length = 0;
				discard_line = true;
				printk("CAL_ERROR,line_too_long\n");
			}
		}

		int64_t now_ms = k_uptime_get();

		if (now_ms - last_report_ms >= CAL_REPORT_PERIOD_MS) {
			struct calibration_telemetry telemetry;

			motor_calibration_get_telemetry(&telemetry);
			last_report_ms = now_ms;
			if (telemetry.calibrating) {
				printk("CAL_DATA,%lld,%d,%d,%d,%d,%d\n", now_ms,
				       (int)telemetry.duty_percent,
				       (int)telemetry.left_rpm, (int)telemetry.right_rpm,
				       (int)telemetry.left_position,
				       (int)telemetry.right_position);
			}
		}
		k_sleep(K_MSEC(CAL_CONSOLE_POLL_MS));
	}
	return 0;
}
