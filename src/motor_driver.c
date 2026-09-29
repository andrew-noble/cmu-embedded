#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>

#include "config.h"
#include "motor_driver.h"

#define MOTOR_NODE DT_PATH(zephyr_user)

static const struct pwm_dt_spec left_pwm = PWM_DT_SPEC_GET_BY_IDX(MOTOR_NODE, 0);
static const struct pwm_dt_spec right_pwm = PWM_DT_SPEC_GET_BY_IDX(MOTOR_NODE, 1);
static const struct gpio_dt_spec left_in1 = GPIO_DT_SPEC_GET(MOTOR_NODE, left_in1_gpios);
static const struct gpio_dt_spec left_in2 = GPIO_DT_SPEC_GET(MOTOR_NODE, left_in2_gpios);
static const struct gpio_dt_spec right_in1 = GPIO_DT_SPEC_GET(MOTOR_NODE, right_in1_gpios);
static const struct gpio_dt_spec right_in2 = GPIO_DT_SPEC_GET(MOTOR_NODE, right_in2_gpios);

static bool drive_active;
static bool drive_reverse;

int motor_driver_coast(void)
{
	/* EN low disables the bridge outputs, allowing the motor to freewheel. */
	drive_active = false;
	int rc = pwm_set_pulse_dt(&left_pwm, 0);

	rc |= pwm_set_pulse_dt(&right_pwm, 0);
	return rc;
}

int motor_driver_brake(void)
{
	/* EN high with both inputs equal gives dynamic braking. */
	drive_active = false;
	int rc = gpio_pin_set_dt(&left_in1, 0);

	rc |= gpio_pin_set_dt(&left_in2, 0);
	rc |= gpio_pin_set_dt(&right_in1, 0);
	rc |= gpio_pin_set_dt(&right_in2, 0);
	rc |= pwm_set_pulse_dt(&left_pwm, left_pwm.period);
	rc |= pwm_set_pulse_dt(&right_pwm, right_pwm.period);
	return rc;
}

int motor_driver_drive(bool reverse, int32_t duty_percent)
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

	uint32_t left_pulse = (uint32_t)((uint64_t)left_pwm.period * duty_percent / 100);
	uint32_t right_pulse = (uint32_t)((uint64_t)right_pwm.period * duty_percent / 100);

	rc = pwm_set_pulse_dt(&left_pwm, left_pulse);
	rc |= pwm_set_pulse_dt(&right_pwm, right_pulse);
	if (rc == 0) {
		drive_active = true;
		drive_reverse = reverse;
	}
	return rc;
}

bool motor_driver_init(void)
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
	return motor_driver_brake() == 0;
}
