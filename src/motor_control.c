#include <zephyr/kernel.h>

#include "config.h"
#include "encoder.h"
#include "motor_control.h"
#include "motor_driver.h"

static struct k_spinlock motor_lock;
static struct motor_command latest_command;
static struct motor_telemetry latest_telemetry;
static uint32_t command_generation;
K_SEM_DEFINE(motor_wake, 0, 1);

bool motor_control_submit(struct motor_command command)
{
	bool accepted = command.valid &&
		command.throttle >= THROTTLE_RAW_MIN &&
		command.throttle <= THROTTLE_RAW_MAX &&
		command.brake >= BRAKE_RAW_MIN &&
		command.brake <= BRAKE_RAW_MAX &&
		command.clutch >= CLUTCH_RAW_MIN &&
		command.clutch <= CLUTCH_RAW_MAX;

	if (!accepted) {
		command = (struct motor_command) { 0 };
	}

	k_spinlock_key_t key = k_spin_lock(&motor_lock);

	latest_command = command;
	command_generation++;
	k_spin_unlock(&motor_lock, key);
	k_sem_give(&motor_wake);
	return accepted;
}


void motor_control_get_telemetry(struct motor_telemetry *telemetry)
{
	k_spinlock_key_t key = k_spin_lock(&motor_lock);

	*telemetry = latest_telemetry;
	k_spin_unlock(&motor_lock, key);
}

static int32_t clamp_duty(int64_t duty)
{
	if (duty <= 0) {
		return 0;
	}
	if (duty > MOTOR_MAX_DUTY_PERCENT) {
		return MOTOR_MAX_DUTY_PERCENT;
	}
	/* Compensate the low-duty region while retaining the full control range.
	 * Telemetry reports the actual PWM sent to the motor driver. */
	return MOTOR_DUTY_OFFSET_PERCENT +
		(int32_t)((duty * (MOTOR_MAX_DUTY_PERCENT - MOTOR_DUTY_OFFSET_PERCENT) +
			   MOTOR_MAX_DUTY_PERCENT / 2) / MOTOR_MAX_DUTY_PERCENT);
}

static int32_t rpm_magnitude(int32_t rpm)
{
	return rpm < 0 ? -rpm : rpm;
}

/* Round up so the 1% deadband still ends at 1% of raw pedal travel. */
static int32_t throttle_percent(int16_t raw)
{
	int32_t travel = (int32_t)THROTTLE_RAW_FULL - THROTTLE_RAW_REST;
	int32_t moved = (int32_t)raw - THROTTLE_RAW_REST;

	if (travel < 0) {
		travel = -travel;
		moved = -moved;
	}
	return CLAMP((moved * 100 + travel - 1) / travel, 0, 100);
}

struct wheel_pid {
	int64_t integral_rpm_ms;
	int32_t previous_measurement;
	int64_t filtered_rate_rpm_s;
	bool measurement_ready;
};

static int32_t wheel_pid_duty(struct wheel_pid *pid, int32_t target_rpm,
			     int32_t measured_rpm, int32_t sample_ms, bool sampled)
{
	int32_t error = target_rpm - measured_rpm;
	int64_t candidate = pid->integral_rpm_ms;
	if (sampled) {
		/* Derivative on measurement avoids a kick on target changes. Keep
		 * the filtered derivative between samples, including command wakes. */
		if (pid->measurement_ready) {
			int64_t rate = ((int64_t)measured_rpm - pid->previous_measurement) *
				1000 / sample_ms;
			pid->filtered_rate_rpm_s +=
				(rate - pid->filtered_rate_rpm_s) * sample_ms /
				(MOTOR_D_FILTER_MS + sample_ms);
		}
		pid->previous_measurement = measured_rpm;
		pid->measurement_ready = true;
		candidate = CLAMP(candidate + (int64_t)error * sample_ms,
			-MOTOR_INTEGRAL_LIMIT_RPM_MS, MOTOR_INTEGRAL_LIMIT_RPM_MS);
	}
	int64_t base = (int64_t)MOTOR_KP_MILLI_PERCENT_PER_RPM * error -
		(int64_t)MOTOR_KD_MILLI_PERCENT_S_PER_RPM * pid->filtered_rate_rpm_s;
	int64_t previous_effort = base +
		MOTOR_KI_MILLI_PERCENT_PER_RPM_S * pid->integral_rpm_ms / 1000;
	/* Permit the step that REACHES saturation. Freeze further accumulation
	 * only once already saturated, so rounding cannot strand output at 99%.
	 * Opposite-sign error can always unwind the integral. */
	if ((previous_effort >= MOTOR_MAX_DUTY_PERCENT * 1000 && error > 0) ||
	    (previous_effort <= 0 && error < 0)) {
		candidate = pid->integral_rpm_ms;
	}
	pid->integral_rpm_ms = candidate;
	int64_t effort = base + MOTOR_KI_MILLI_PERCENT_PER_RPM_S * candidate / 1000;
	return clamp_duty(effort / 1000);
}

static bool pedal_pressed(int32_t raw, int32_t threshold, bool pressed_high)
{
	return pressed_high ? raw >= threshold : raw <= threshold;
}

BUILD_ASSERT(THROTTLE_RAW_REST != THROTTLE_RAW_FULL);
BUILD_ASSERT(THROTTLE_RAW_MIN <= THROTTLE_RAW_REST &&
	     THROTTLE_RAW_REST <= THROTTLE_RAW_MAX);
BUILD_ASSERT(THROTTLE_RAW_MIN <= THROTTLE_RAW_FULL &&
	     THROTTLE_RAW_FULL <= THROTTLE_RAW_MAX);
BUILD_ASSERT(BRAKE_RAW_MIN <= BRAKE_PRESSED_AT &&
	     BRAKE_PRESSED_AT <= BRAKE_RAW_MAX);
BUILD_ASSERT(CLUTCH_RAW_MIN <= CLUTCH_PRESSED_AT &&
	     CLUTCH_PRESSED_AT <= CLUTCH_RAW_MAX);
BUILD_ASSERT(MOTOR_MAX_DUTY_PERCENT > 0 && MOTOR_MAX_DUTY_PERCENT <= 100);
BUILD_ASSERT(MOTOR_MAX_RPM >= 2);
BUILD_ASSERT(MOTOR_D_FILTER_MS >= 0);
BUILD_ASSERT(MOTOR_KP_MILLI_PERCENT_PER_RPM >= 0 &&
	     MOTOR_KI_MILLI_PERCENT_PER_RPM_S >= 0 &&
	     MOTOR_KD_MILLI_PERCENT_S_PER_RPM >= 0);
BUILD_ASSERT(MOTOR_DUTY_OFFSET_PERCENT >= 0 &&
	     MOTOR_DUTY_OFFSET_PERCENT < MOTOR_MAX_DUTY_PERCENT);

static void control_tick(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	k_sem_give(&motor_wake);
}

K_TIMER_DEFINE(control_timer, control_tick, NULL);

static void motor_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	struct encoder_sample sample = { 0 };
	struct motor_command command = { 0 };
	uint32_t seen_generation = 0;
	int64_t last_sample_ms = k_uptime_get();
	struct wheel_pid left_pid = {0}, right_pid = {0};
	int32_t left_duty = 0, right_duty = 0;
	int32_t left_rpm = 0, right_rpm = 0, target_rpm = 0, duty = 0;
	bool reverse = false, clutch_was_pressed = false, motor_fault = false;
	bool sensors_healthy = true;
	bool hardware_ok = motor_driver_init();

	if (!hardware_ok) {
		printk("Motor hardware not ready, check overlay\n");
		return;
	}

	if (encoder_init() != 0) {
		return;
	}

	/* A periodic kernel timer gives a fixed period that does not drift,
	 * unlike k_sleep() after each iteration. */
	k_timer_start(&control_timer, K_MSEC(ENC_PERIOD_MS), K_MSEC(ENC_PERIOD_MS));

	while (1) {
		k_sem_take(&motor_wake, K_FOREVER);

		k_spinlock_key_t key = k_spin_lock(&motor_lock);
		bool new_command = seen_generation != command_generation;

		command = latest_command;
		seen_generation = command_generation;
		k_spin_unlock(&motor_lock, key);

		uint32_t ticks = k_timer_status_get(&control_timer);
		int32_t sample_ms = ENC_PERIOD_MS;
		if (ticks > 0) {
			int64_t now = k_uptime_get();

			sample_ms = (int32_t)(now - last_sample_ms);
			last_sample_ms = now;
			if (sample_ms <= 0) {
				sample_ms = ENC_PERIOD_MS;
			}
			sensors_healthy = encoder_sample(sample_ms, &sample) == 0;
			if (sensors_healthy) {
				left_rpm = sample.left_rpm;
				right_rpm = sample.right_rpm;
			}
		}

		int32_t throttle = throttle_percent(command.throttle);
		bool clutch_pressed = CLUTCH_CONTROL_ENABLED &&
			pedal_pressed(command.clutch, CLUTCH_PRESSED_AT,
				      CLUTCH_PRESSED_HIGH);
		bool brake_pressed = pedal_pressed(command.brake, BRAKE_PRESSED_AT,
						BRAKE_PRESSED_HIGH);

		/* A press toggles direction once, only with zero throttle and stopped wheels. */
		if (new_command) {
			if (command.valid && clutch_pressed && !clutch_was_pressed &&
			    throttle <= THROTTLE_ZERO_PERCENT &&
			    rpm_magnitude(left_rpm) <= MOTOR_SHIFT_MAX_RPM &&
			    rpm_magnitude(right_rpm) <= MOTOR_SHIFT_MAX_RPM) {
				reverse = !reverse;
				left_pid = (struct wheel_pid){0};
				right_pid = (struct wheel_pid){0};
			}
			clutch_was_pressed = clutch_pressed;
		}

		if (motor_fault || !sensors_healthy || !command.valid || brake_pressed ||
		    !MOTOR_OUTPUTS_ENABLED) {
			target_rpm = 0;
			duty = 0;
			left_pid = (struct wheel_pid){0};
			right_pid = (struct wheel_pid){0};
			left_duty = right_duty = 0;
			if (motor_driver_brake() != 0) {
				motor_fault = true;
			}
		} else if (throttle <= THROTTLE_ZERO_PERCENT) {
			/* Released accelerator: suspend PID and let the wheels coast. */
			target_rpm = 0;
			duty = 0;
			left_pid = (struct wheel_pid){0};
			right_pid = (struct wheel_pid){0};
			left_duty = right_duty = 0;
			if (motor_driver_coast() != 0) {
				motor_fault = true;
				motor_driver_brake();
			}
		} else if (ticks > 0 || new_command) {
			/* Independent feedback: a moving wheel cannot hide a stopped one. */
			target_rpm = throttle * MOTOR_MAX_RPM / 100;
			left_duty = wheel_pid_duty(&left_pid, target_rpm,
				reverse ? -left_rpm : left_rpm, sample_ms, ticks > 0);
			right_duty = wheel_pid_duty(&right_pid, target_rpm,
				reverse ? -right_rpm : right_rpm, sample_ms, ticks > 0);
			duty = (left_duty + right_duty) / 2;

			if (MOTOR_OUTPUTS_ENABLED && (left_duty > 0 || right_duty > 0)) {
				if (motor_driver_drive_wheels(reverse, left_duty, right_duty) != 0) {
					motor_fault = true;
					motor_driver_brake();
				}
			} else if (motor_driver_coast() != 0) {
				/* Zero PID drive coasts; an output failure still brakes. */
				motor_fault = true;
				motor_driver_brake();
			}
		}

		key = k_spin_lock(&motor_lock);
		latest_telemetry = (struct motor_telemetry) {
			.left_position = sample.left_position,
			.right_position = sample.right_position,
			.left_rpm = left_rpm,
			.right_rpm = right_rpm,
			.target_rpm = target_rpm,
			.duty_percent = duty,
			.left_duty_percent = left_duty,
			.right_duty_percent = right_duty,
			.reverse = reverse,
		};
		k_spin_unlock(&motor_lock, key);
	}
}

K_THREAD_DEFINE(motor_tid, MOTOR_STACK_SIZE, motor_thread,
		NULL, NULL, NULL, MOTOR_PRIORITY, 0, 0);
