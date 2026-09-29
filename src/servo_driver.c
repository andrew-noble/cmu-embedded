#include <zephyr/kernel.h>
#include <zephyr/drivers/pwm.h>

#include "config.h"
#include "servo_driver.h"

static const struct pwm_dt_spec servo_pwm =
	PWM_DT_SPEC_GET_BY_NAME(DT_PATH(zephyr_user), steering);

BUILD_ASSERT(SERVO_PULSE_MIN_US > 0 && SERVO_PULSE_MIN_US < SERVO_PULSE_MAX_US);
BUILD_ASSERT(SERVO_CENTER_US >= SERVO_PULSE_MIN_US &&
	     SERVO_CENTER_US <= SERVO_PULSE_MAX_US);
BUILD_ASSERT(SERVO_LEFT_US >= SERVO_PULSE_MIN_US && SERVO_LEFT_US <= SERVO_PULSE_MAX_US);
BUILD_ASSERT(SERVO_RIGHT_US >= SERVO_PULSE_MIN_US && SERVO_RIGHT_US <= SERVO_PULSE_MAX_US);
BUILD_ASSERT(STEER_RAW_MIN < 0 && STEER_RAW_MAX > 0);
BUILD_ASSERT(STEER_DEAD_ZONE >= 0 && STEER_DEAD_ZONE < STEER_RAW_MAX &&
	     STEER_DEAD_ZONE < -STEER_RAW_MIN);

int servo_driver_disable(void)
{
	return pwm_set_pulse_dt(&servo_pwm, 0);
}

int servo_driver_init(void)
{
	if (!pwm_is_ready_dt(&servo_pwm)) {
		return -ENODEV;
	}
	return servo_driver_disable();
}

int servo_driver_set_pulse(uint32_t pulse_us)
{
	if (pulse_us < SERVO_PULSE_MIN_US || pulse_us > SERVO_PULSE_MAX_US) {
		return -EINVAL;
	}
	return pwm_set_pulse_dt(&servo_pwm, PWM_USEC(pulse_us));
}

int servo_driver_set_steering(int16_t steering)
{
	if (steering < STEER_RAW_MIN || steering > STEER_RAW_MAX) {
		return -EINVAL;
	}

	/* Hold center throughout the dead zone; rescale each remaining half so
	 * there is no position jump at its edge. Signed math permits reversal. */
	int32_t pulse = SERVO_CENTER_US;
	if (steering < -STEER_DEAD_ZONE) {
		pulse += ((int32_t)steering + STEER_DEAD_ZONE) *
			 (SERVO_LEFT_US - SERVO_CENTER_US) /
			 (STEER_RAW_MIN + STEER_DEAD_ZONE);
	} else if (steering > STEER_DEAD_ZONE) {
		pulse += ((int32_t)steering - STEER_DEAD_ZONE) *
			 (SERVO_RIGHT_US - SERVO_CENTER_US) /
			 (STEER_RAW_MAX - STEER_DEAD_ZONE);
	}
	return servo_driver_set_pulse((uint32_t)pulse);
}
