#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/sensor.h>

#define UART_DEVICE_NODE DT_NODELABEL(usart1)

/* Frame format: must match bridge.c on the Pi */
#define UART_SOF        0xA5
#define FRAME_LEN       13      /* SOF + ID(2) + DLC + data(8) + CRC */
#define CMD_ID_HI       0x01    /* ID 0x100 */
#define CMD_ID_LO       0x00
#define CMD_DLC         8
#define LINK_TIMEOUT_MS 150     /* time since last VALID frame */
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


static uint8_t crc8(const uint8_t *buf, size_t len) /* CRC-8 checksum, poly 0x07, init 0x00. Identical to the Pi's version. */
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


/* ------------------------------------------------------------------------
 * Encoders
 * Each encoder is counted in hardware by a timer in quadrature (x4) mode.
 * A thread samples both counters every ENC_PERIOD_MS and turns the change
 * in count into a velocity.
 * ---------------------------------------------------------------------- */

#define ENC_PERIOD_MS    10      /* sampling period */
#define ENC_PRINT_EVERY  10      /* print every N samples (100 ms) */
#define ENC_STACK_SIZE   1024
#define ENC_PRIORITY     2       /* revisit when building the task table */

#define QDEC_LEFT_NODE   DT_NODELABEL(qdec_left)
#define QDEC_RIGHT_NODE  DT_NODELABEL(qdec_right)

/* The STM32 qdec driver makes the 16-bit counter wrap at the largest
 * multiple of counts-per-revolution that fits: 65535 - (65535 % cpr).
 * We need the same number to undo the wrap. */
#define ENC_MODULUS(cpr) (65535 - (65535 % (cpr)))

struct encoder {
	const char *name;
	const struct device *device;
	int32_t cpr;        /* counts per wheel revolution (from devicetree) */
	int32_t modulus;    /* counter wraps back to 0 at this value */
	int32_t last_raw;   /* counter value at the previous sample */
	int32_t position;   /* accumulated counts since boot */
	int32_t delta;      /* counts during the most recent period */
};

static struct encoder enc_left = {
	.name = "L",
	.device = DEVICE_DT_GET(QDEC_LEFT_NODE),
	.cpr = DT_PROP(QDEC_LEFT_NODE, st_counts_per_revolution),
	.modulus = ENC_MODULUS(DT_PROP(QDEC_LEFT_NODE, st_counts_per_revolution)),
};

static struct encoder enc_right = {
	.name = "R",
	.device = DEVICE_DT_GET(QDEC_RIGHT_NODE),
	.cpr = DT_PROP(QDEC_RIGHT_NODE, st_counts_per_revolution),
	.modulus = ENC_MODULUS(DT_PROP(QDEC_RIGHT_NODE, st_counts_per_revolution)),
};


static int encoder_read_raw(const struct encoder *e, int32_t *raw) /* Read the raw hardware counter through the sensor API. */
{
	struct sensor_value v;
	int rc = sensor_sample_fetch_chan(e->device, SENSOR_CHAN_ENCODER_COUNT);

	if (rc == 0) {
		rc = sensor_channel_get(e->device, SENSOR_CHAN_ENCODER_COUNT, &v); /*replace rc*/
	}
	if (rc == 0) { 	/*check rc again*/
		*raw = v.val1;
	}
	return rc;
}


static void encoder_update(struct encoder *e) /* Sample one encoder: change since last sample, with wraparound undone. */
{
	int32_t raw;

	if (encoder_read_raw(e, &raw) != 0) {
		return;
	}

	int32_t d = raw - e->last_raw;

	/* A jump of more than half the counter range means it wrapped. */
	if (d > e->modulus / 2) {
		d -= e->modulus;
	} else if (d < -(e->modulus / 2)) {
		d += e->modulus;
	}

	e->last_raw = raw;
	e->delta = d;
	e->position += d;
}


static int32_t counts_to_rpm(int32_t counts, int32_t cpr, int32_t window_ms) /* Wheel speed in rpm over a window of `counts` taken in `window_ms`. */
{
	return (int32_t)((int64_t)counts * 60000 / ((int64_t)cpr * window_ms));
}


/*------------------------------------------------------------ THREAD ---------------------------------------------------------------*/
K_TIMER_DEFINE(enc_timer, NULL, NULL);


static void encoder_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	struct encoder *encs[] = { &enc_left, &enc_right };
	int32_t window_start[2];
	uint32_t n = 0;

	for (int i = 0; i < 2; i++) {
		if (!device_is_ready(encs[i]->device) ||
		    encoder_read_raw(encs[i], &encs[i]->last_raw) != 0) {
			printk("Encoder %s not ready, check overlay\n", encs[i]->name);
			return;
		}
		window_start[i] = 0;
	}

	printk("Encoders ready (cpr L=%d R=%d, period %d ms)\n",
	       (int)enc_left.cpr, (int)enc_right.cpr, ENC_PERIOD_MS);

	/* A periodic kernel timer gives a fixed period that does not drift,
	 * unlike k_sleep() after each iteration. */
	k_timer_start(&enc_timer, K_MSEC(ENC_PERIOD_MS), K_MSEC(ENC_PERIOD_MS));

	while (1) {
		k_timer_status_sync(&enc_timer);   /* wait for the next tick */

		encoder_update(&enc_left);
		encoder_update(&enc_right);

		if (++n % ENC_PRINT_EVERY == 0) {
			int32_t window_ms = ENC_PERIOD_MS * ENC_PRINT_EVERY;
			int32_t rpm[2];

			for (int i = 0; i < 2; i++) {
				int32_t moved = encs[i]->position - window_start[i];

				rpm[i] = counts_to_rpm(moved, encs[i]->cpr, window_ms);
				window_start[i] = encs[i]->position;
			}

			printk("ENC  L pos %8d  rpm %5d  |  R pos %8d  rpm %5d\n",
			       (int)enc_left.position, (int)rpm[0],
			       (int)enc_right.position, (int)rpm[1]);
		}
	}
}

K_THREAD_DEFINE(enc_tid, ENC_STACK_SIZE, encoder_thread, NULL, NULL, NULL,
			ENC_PRIORITY, 0, 0);
/*------------------------------------------------------------ THREAD ---------------------------------------------------------------*/


int main(void)
{
	bool link_ok = false;           /* power up in the "link down" state */
	int64_t last_valid_ms = 0;
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
				last_valid_ms = k_uptime_get();
				n_valid++;

				if (!link_ok) {
					link_ok = true;
					printk("LINK UP\n");
				}

				if (n_valid % PRINT_EVERY == 0) {
					int16_t  steer = (int16_t)(uint16_t)(frame[4] | (frame[5] << 8));
					uint16_t thr   = (uint16_t)(frame[6] | (frame[7] << 8));
					uint16_t brk   = (uint16_t)(frame[8] | (frame[9] << 8));

					printk("steer %6d  thr %5u  brk %5u  seq %3u  bad %u\n",
					       steer, (unsigned)thr, (unsigned)brk,
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
				printk("LINK LOST (%d ms since last valid frame)\n",
				       (int)elapsed);
			}
		}
	}
	return 0;
}
