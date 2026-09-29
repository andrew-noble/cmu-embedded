#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

#include "testpoint.h"

#define TP_NODE DT_PATH(zephyr_user)

/* A missing property gives a spec with port == NULL, which is skipped. */
static const struct gpio_dt_spec testpoints[TP_COUNT] = {
	[TP_CMD_RX]  = GPIO_DT_SPEC_GET_OR(TP_NODE, tp_cmd_rx_gpios, {0}),
	[TP_PWM_SET] = GPIO_DT_SPEC_GET_OR(TP_NODE, tp_pwm_set_gpios, {0}),
};

int testpoint_init(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(testpoints); i++) {
		if (testpoints[i].port == NULL) {
			continue;
		}
		if (!gpio_is_ready_dt(&testpoints[i]) ||
		    gpio_pin_configure_dt(&testpoints[i], GPIO_OUTPUT_INACTIVE) != 0) {
			return -ENODEV;
		}
	}
	return 0;
}

void testpoint_toggle(enum testpoint tp)
{
	if (tp < TP_COUNT && testpoints[tp].port != NULL) {
		gpio_pin_toggle_dt(&testpoints[tp]);
	}
}
