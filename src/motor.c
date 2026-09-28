#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>

#include "encoder.h"
#include "motor.h"
#include "motor_config.h"

/* L298N outputs and closed-loop control live together in this module. */
#define MOTOR_NODE DT_PATH(zephyr_user)

static const struct pwm_dt_spec left_pwm = PWM_DT_SPEC_GET_BY_IDX(MOTOR_NODE, 0);
static const struct pwm_dt_spec right_pwm = PWM_DT_SPEC_GET_BY_IDX(MOTOR_NODE, 1);
static const struct gpio_dt_spec left_in1 = GPIO_DT_SPEC_GET(MOTOR_NODE, left_in1_gpios);
static const struct gpio_dt_spec left_in2 = GPIO_DT_SPEC_GET(MOTOR_NODE, left_in2_gpios);
static const struct gpio_dt_spec right_in1 = GPIO_DT_SPEC_GET(MOTOR_NODE, right_in1_gpios);
static const struct gpio_dt_spec right_in2 = GPIO_DT_SPEC_GET(MOTOR_NODE, right_in2_gpios);

static struct k_spinlock motor_lock;
static struct motor_command latest_command;
static struct motor_telemetry latest_telemetry;
static uint32_t command_generation;
static bool drive_active;
static bool drive_reverse;
K_SEM_DEFINE(motor_wake, 0, 1);

void motor_submit(struct motor_command command)
{
	k_spinlock_key_t key = k_spin_lock(&motor_lock);

	latest_command = command;
	command_generation++;
	k_spin_unlock(&motor_lock, key);
	k_sem_give(&motor_wake);
}


void motor_get_telemetry(struct motor_telemetry *telemetry)
{
	k_spinlock_key_t key = k_spin_lock(&motor_lock);

	*telemetry = latest_telemetry;
	k_spin_unlock(&motor_lock, key);
}

static int motor_brake(void)
{
	/* L298N: EN high with both inputs equal gives dynamic braking.
	 * EN low would let the wheel coast, so "PWM disabled" means constant high.
	 */
	drive_active = false;
	int rc = gpio_pin_set_dt(&left_in1, 0);

	rc |= gpio_pin_set_dt(&left_in2, 0);
	rc |= gpio_pin_set_dt(&right_in1, 0);
	rc |= gpio_pin_set_dt(&right_in2, 0);
	rc |= pwm_set_pulse_dt(&left_pwm, left_pwm.period);
	rc |= pwm_set_pulse_dt(&right_pwm, right_pwm.period);
	return rc;
}

static int motor_drive(bool reverse, int32_t duty_permille)
{
	int left_forward = LEFT_FORWARD_IN1_HIGH;
	int right_forward = RIGHT_FORWARD_IN1_HIGH;

	if (reverse) {
		left_forward = !left_forward;
		right_forward = !right_forward;
	}

	int rc = 0;

	if (!drive_active || drive_reverse != reverse) {
		/* Disable enables before a direction change. */
		rc = pwm_set_pulse_dt(&left_pwm, 0);
		rc |= pwm_set_pulse_dt(&right_pwm, 0);
		rc |= gpio_pin_set_dt(&left_in1, left_forward);
		rc |= gpio_pin_set_dt(&left_in2, !left_forward);
		rc |= gpio_pin_set_dt(&right_in1, right_forward);
		rc |= gpio_pin_set_dt(&right_in2, !right_forward);
		if (rc != 0) {
			return rc;
		}
	}

	uint32_t left_pulse = (uint32_t)((uint64_t)left_pwm.period * duty_permille / 1000);
	uint32_t right_pulse = (uint32_t)((uint64_t)right_pwm.period * duty_permille / 1000);

	rc = pwm_set_pulse_dt(&left_pwm, left_pulse);
	rc |= pwm_set_pulse_dt(&right_pwm, right_pulse);
	if (rc == 0) {
		drive_active = true;
		drive_reverse = reverse;
	}
	return rc;
}

static bool motor_hardware_ready(void)
{
	const struct gpio_dt_spec *pins[] = {
		&left_in1, &left_in2, &right_in1, &right_in2
	};

	if (!pwm_is_ready_dt(&left_pwm) || !pwm_is_ready_dt(&right_pwm)) {
		return false;
	}
	for (size_t i = 0; i < ARRAY_SIZE(pins); i++) {
		if (!gpio_is_ready_dt(pins[i]) ||
		    gpio_pin_configure_dt(pins[i], GPIO_OUTPUT_INACTIVE) != 0) {
			return false;
		}
	}
	return motor_brake() == 0;
}

static int32_t clamp_duty(int64_t duty)
{
	if (duty < 0) {
		return 0;
	}
	if (duty > 1000) {
		return 1000;
	}
	return (int32_t)duty;
}

static int32_t rpm_magnitude(int32_t rpm)
{
	return rpm < 0 ? -rpm : rpm;
}

/* Convert either increasing or decreasing throttle readings to 0..1000. */
static int32_t throttle_permille(uint16_t raw)
{
	int32_t travel = (int32_t)THROTTLE_RAW_FULL - THROTTLE_RAW_REST;
	int32_t moved = (int32_t)raw - THROTTLE_RAW_REST;

	if (travel < 0) {
		travel = -travel;
		moved = -moved;
	}
	return CLAMP(moved * 1000 / travel, 0, 1000);
}

static bool pedal_pressed(uint16_t raw, uint16_t threshold, bool pressed_high)
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
	int64_t integral_rpm_ms = 0;
	int32_t previous_error = 0;
	int32_t left_rpm = 0, right_rpm = 0, target_rpm = 0, duty = 0;
	bool reverse = false, clutch_was_pressed = false, motor_fault = false;
	bool sensors_healthy = true;
	bool hardware_ok = motor_hardware_ready();

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

		int32_t throttle = throttle_permille(command.throttle);
		bool clutch_pressed = pedal_pressed(command.clutch, CLUTCH_PRESSED_AT,
						CLUTCH_PRESSED_HIGH);
		bool brake_pressed = pedal_pressed(command.brake, BRAKE_PRESSED_AT,
						BRAKE_PRESSED_HIGH);

		/* A press toggles direction once, only with zero throttle and stopped wheels. */
		if (new_command) {
			if (command.valid && clutch_pressed && !clutch_was_pressed &&
			    throttle <= THROTTLE_ZERO_PERMILLE &&
			    rpm_magnitude(left_rpm) <= MOTOR_SHIFT_MAX_RPM &&
			    rpm_magnitude(right_rpm) <= MOTOR_SHIFT_MAX_RPM) {
				reverse = !reverse;
				integral_rpm_ms = 0;
				previous_error = 0;
			}
			clutch_was_pressed = clutch_pressed;
		}

		if (motor_fault || !sensors_healthy || !command.valid || brake_pressed ||
		    throttle <= THROTTLE_ZERO_PERMILLE) {
			target_rpm = 0;
			duty = 0;
			integral_rpm_ms = 0;
			previous_error = 0;
			if (motor_brake() != 0) {
				motor_fault = true;
			}
		} else if (ticks > 0 || new_command) {
			/* Both encoders contribute equally to the measured wheel speed. */
			int32_t measured_rpm = (left_rpm + right_rpm) / 2;
			int32_t directional_rpm = reverse ? -measured_rpm : measured_rpm;

			target_rpm = throttle * MOTOR_MAX_RPM / 1000;
			int32_t error = target_rpm - directional_rpm;

			if (ticks > 0) {
				integral_rpm_ms += (int64_t)error * sample_ms;
				integral_rpm_ms = CLAMP(integral_rpm_ms, -500000LL, 500000LL);
			}

			int64_t correction = (int64_t)MOTOR_KP_PERMILLE_PER_RPM * error +
				(int64_t)MOTOR_KI_PERMILLE_PER_RPM_S * integral_rpm_ms / 1000;

			if (ticks > 0) {
				correction += (int64_t)MOTOR_KD_PERMILLE_S_PER_RPM *
					(error - previous_error) * 1000 / sample_ms;
				previous_error = error;
			}
			duty = clamp_duty((int64_t)MOTOR_FF_MAX_PERMILLE * target_rpm /
					  MOTOR_MAX_RPM + correction);
			if (MOTOR_OUTPUTS_ENABLED && duty > 0) {
				if (motor_drive(reverse, duty) != 0) {
					motor_fault = true;
					motor_brake();
				}
			} else {
				motor_brake();
			}
		}

		key = k_spin_lock(&motor_lock);
		latest_telemetry = (struct motor_telemetry) {
			.left_position = sample.left_position,
			.right_position = sample.right_position,
			.left_rpm = left_rpm,
			.right_rpm = right_rpm,
			.target_rpm = target_rpm,
			.duty_permille = duty,
			.reverse = reverse,
		};
		k_spin_unlock(&motor_lock, key);
	}
}

K_THREAD_DEFINE(motor_tid, 2048, motor_thread, NULL, NULL, NULL, 0, 0, 0);
