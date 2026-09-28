#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>

#include "motor.h"
#include "motor_config.h"

/* Pi link: interrupt receives bytes; main parses complete commands. */
#define UART_DEVICE_NODE DT_NODELABEL(usart1)

/* Provisional frame format; verify against the Pi sender. */
#define UART_SOF        0xA5
#define FRAME_LEN       13      /* SOF + ID(2) + DLC + data(8) + CRC */
#define CMD_ID_HI       0x01    /* ID 0x100 */
#define CMD_ID_LO       0x00
#define CMD_DLC         8
#define LINK_TIMEOUT_MS 95      /* checked every 5 ms; meet 100 ms checkoff */
#define PRINT_EVERY     10      /* print 1 of every N valid frames */

K_MSGQ_DEFINE(rx_q, sizeof(uint8_t), 64, 1);  /* Raw bytes from the ISR to the parser. A full queue drops new bytes. */
static const struct device *const uart_device = DEVICE_DT_GET(UART_DEVICE_NODE); /* Pointer to Zephyr's USART1 device object. */


static void serial_cb(const struct device *device, void *user_data) /* ISR: move bytes from the hardware FIFO into the queue. Nothing else. */
{
	ARG_UNUSED(user_data);

	uint8_t c;
	uart_irq_update(device);
	if (uart_irq_rx_ready(device) <= 0) {
		return;
	}
	while (uart_fifo_read(device, &c, 1) == 1) {
		k_msgq_put(&rx_q, &c, K_NO_WAIT);
	}
}


static uint8_t crc8(const uint8_t *buf, size_t len) /* CRC-8, poly 0x07, init 0x00. */
{
	uint8_t crc = 0x00;

	for (size_t i = 0; i < len; i++) {
		crc ^= buf[i];
		for (int b = 0; b < 8; b++) {
			crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07)
					   : (uint8_t)(crc << 1);
		}
	}
	return crc;
}

/* Parser state: only the main thread touches these. */
static uint8_t frame[FRAME_LEN];
static int idx;


static int parse_byte(uint8_t c) /* Feed one byte. Returns 1 for a complete valid frame (in frame[]), * -1 for a rejected frame, 0 if more bytes are needed. */
{
	if (idx == 0) {
		if (c == UART_SOF) {
			frame[idx++] = c;
		}
		return 0;
	}

	frame[idx++] = c;

	if ((idx == 2 && c != CMD_ID_HI) ||  /* Check the header early so a false start byte costs little. * If the bad byte is itself a start byte, begin a new frame with it. */
	    (idx == 3 && c != CMD_ID_LO) ||
	    (idx == 4 && c != CMD_DLC)) {
		idx = 0;
		if (c == UART_SOF) {
			frame[idx++] = c;
		}
		return -1;
	}

	if (idx < FRAME_LEN) {
		return 0;
	}

	idx = 0;
	return (crc8(&frame[1], FRAME_LEN - 2) == frame[FRAME_LEN - 1]) ? 1 : -1;
}


int main(void)
{
	bool link_ok = false;           /* power up in the "link down" state */
	int64_t last_valid_ms = 0;
	int64_t last_print_ms = 0;
	uint32_t n_valid = 0, n_bad = 0;

	if (!device_is_ready(uart_device)) {
		printk("USART1 not ready, check overlay\n");
		return 0;
	}

	int ret = uart_irq_callback_user_data_set(uart_device, serial_cb, NULL);

	if (ret < 0) {
		printk("Error setting UART callback: %d\n", ret);
		return 0;
	}

	uart_irq_rx_enable(uart_device);
	printk("CRC self-test: 0x%02x (expect 0xf4)\n", crc8((const uint8_t *)"123456789", 9));
	printk("Waiting for commands (link DOWN)\n");

	while (1) {
		uint8_t c;

		/* Wait at most 5 ms for a byte, so the link check below
		 * still runs when nothing is arriving. */
		if (k_msgq_get(&rx_q, &c, K_MSEC(5)) == 0) {
			int r = parse_byte(c);

			if (r > 0) {
				int16_t steer = (int16_t)(uint16_t)(frame[4] | (frame[5] << 8));
				struct motor_command command = {
					.throttle = (uint16_t)(frame[6] | (frame[7] << 8)),
					.brake = (uint16_t)(frame[8] | (frame[9] << 8)),
					.clutch = frame[10],
					.valid = true,
				};

				if (command.throttle < THROTTLE_RAW_MIN ||
				    command.throttle > THROTTLE_RAW_MAX ||
				    command.brake < BRAKE_RAW_MIN ||
				    command.brake > BRAKE_RAW_MAX ||
				    command.clutch < CLUTCH_RAW_MIN ||
				    command.clutch > CLUTCH_RAW_MAX) {
					n_bad++;
					motor_submit((struct motor_command) { 0 });
					link_ok = false;
					continue;
				}

				last_valid_ms = k_uptime_get();
				n_valid++;
				motor_submit(command);

				if (!link_ok) {
					link_ok = true;
					printk("LINK UP\n");
				}

				if (n_valid % PRINT_EVERY == 0) {
					printk("steer %6d  thr %5u  brk %5u  clutch %3u  seq %3u  bad %u\n",
					       steer, (unsigned)command.throttle,
					       (unsigned)command.brake, (unsigned)command.clutch,
					       (unsigned)frame[11], (unsigned)n_bad);
				}
			} else if (r < 0) {
				n_bad++;
			}
		}

		if (link_ok) {
			int64_t elapsed = k_uptime_get() - last_valid_ms;

			if (elapsed > LINK_TIMEOUT_MS) {
				link_ok = false;
				motor_submit((struct motor_command) { 0 });
				printk("LINK LOST (%d ms since last valid frame)\n",
				       (int)elapsed);
			}
		}

		if (k_uptime_get() - last_print_ms >= 100) {
			struct motor_telemetry telemetry;
			motor_get_telemetry(&telemetry);
			last_print_ms = k_uptime_get();
			printk("ENC L %8d %5d rpm | R %8d %5d rpm | target %3d duty %4d %s\n",
			       (int)telemetry.left_position, (int)telemetry.left_rpm,
			       (int)telemetry.right_position, (int)telemetry.right_rpm,
			       (int)telemetry.target_rpm, (int)telemetry.duty_permille,
			       telemetry.reverse ? "REV" : "FWD");
		}
	}
	return 0;
}
