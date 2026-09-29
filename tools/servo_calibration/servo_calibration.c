#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/atomic.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"
#include "motor_driver.h"
#include "servo_driver.h"
#include "servo_calibration_config.h"

static const struct device *const console = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
K_MSGQ_DEFINE(rx_bytes, sizeof(uint8_t), SERVO_CAL_RX_BYTES, 1);
static atomic_t rx_overflow;
static bool armed;
static bool fault;
static int64_t deadline;
static uint32_t pulse_us;

static void serial_cb(const struct device *device, void *unused)
{
	ARG_UNUSED(unused);
	uint8_t c;
	uart_irq_update(device);
	if (!uart_irq_rx_ready(device)) {
		return;
	}
	while (uart_fifo_read(device, &c, 1) == 1) {
		if (k_msgq_put(&rx_bytes, &c, K_NO_WAIT) != 0) {
			atomic_set(&rx_overflow, 1);
		}
	}
}

static void stop(void)
{
	armed = false;
	if (servo_driver_disable() != 0) {
		fault = true;
		printk("SERVO_ERROR,disable_failed\n");
	} else {
		printk("SERVO_OFF\n");
	}
}

static void set_pulse(uint32_t requested)
{
	if (servo_driver_set_pulse(requested) != 0) {
		fault = true;
		stop();
		printk("SERVO_ERROR,pwm_failed\n");
		return;
	}
	pulse_us = requested;
	deadline = k_uptime_get() + SERVO_CAL_TIMEOUT_MS;
	printk("SERVO_PULSE,%u\n", (unsigned)pulse_us);
}

static void handle_line(const char *line)
{
	if (strcmp(line, "SERVO STOP") == 0) {
		stop();
	} else if (strcmp(line, "SERVO ARM") == 0 && !fault) {
		armed = true;
		set_pulse(SERVO_CENTER_US);
		if (armed) {
			printk("SERVO_ARMED,%u\n", (unsigned)pulse_us);
		}
	} else if (strcmp(line, "SERVO KEEP") == 0 && armed) {
		deadline = k_uptime_get() + SERVO_CAL_TIMEOUT_MS;
		printk("SERVO_ALIVE,%u\n", (unsigned)pulse_us);
	} else if (strncmp(line, "SERVO PULSE ", 12) == 0 && armed) {
		char *end;
		const char *number = line + 12;
		unsigned long value = strtoul(number, &end, 10);
		if (*number < '0' || *number > '9' || *end != '\0' ||
		    value < SERVO_PULSE_MIN_US || value > SERVO_PULSE_MAX_US) {
			stop();
			printk("SERVO_ERROR,invalid_pulse\n");
		} else {
			set_pulse(value);
		}
	} else {
		stop();
		printk("SERVO_ERROR,invalid_command_or_not_armed\n");
	}
}

int main(void)
{
	/* This build has no Pi command handler or motor-control thread. */
	if (!motor_driver_init() || motor_driver_coast() != 0 ||
	    servo_driver_init() != 0 || !device_is_ready(console)) {
		printk("SERVO_ERROR,hardware_not_ready\n");
		return 0;
	}
	if (uart_irq_callback_user_data_set(console, serial_cb, NULL) != 0) {
		printk("SERVO_ERROR,uart_callback\n");
		return 0;
	}
	uart_irq_rx_enable(console);
	printk("SERVO_READY,center=%u,min=%u,max=%u\n",
	       SERVO_CENTER_US, SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US);
	char line[SERVO_CAL_LINE_BYTES];
	size_t length = 0;
	bool discard = false;
	while (1) {
		uint8_t c;
		int rc = k_msgq_get(&rx_bytes, &c, K_MSEC(10));
		if (armed && k_uptime_get() >= deadline) {
			stop();
			printk("SERVO_TIMEOUT\n");
		}
		if (atomic_clear(&rx_overflow)) {
			stop();
			length = 0;
			discard = true;
			printk("SERVO_ERROR,rx_overflow\n");
		}
		if (rc != 0 || c == '\r') {
			continue;
		}
		if (c == '\n') {
			line[length] = '\0';
			if (!discard && length > 0) {
				handle_line(line);
			}
			length = 0;
			discard = false;
		} else if (!discard) {
			if (length < sizeof(line) - 1) {
				line[length++] = c;
			} else {
				stop();
				length = 0;
				discard = true;
				printk("SERVO_ERROR,line_too_long\n");
			}
		}
	}
}
