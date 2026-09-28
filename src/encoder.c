#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

#include "encoder.h"

/* Hardware timers count both encoders in quadrature (x4) mode.
 * This module samples the counters and calculates wheel velocity. */

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
		rc = sensor_channel_get(e->device, SENSOR_CHAN_ENCODER_COUNT, &v);
	}
	if (rc == 0) {
		*raw = v.val1;
	}
	return rc;
}


static int encoder_update(struct encoder *e) /* Sample one encoder: change since last sample, with wraparound undone. */
{
	int32_t raw;

	if (encoder_read_raw(e, &raw) != 0) {
		return -EIO;
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
	return 0;
}


static int32_t counts_to_rpm(int32_t counts, int32_t cpr, int32_t window_ms) /* Wheel speed in rpm over a window of `counts` taken in `window_ms`. */
{
	return (int32_t)((int64_t)counts * 60000 / ((int64_t)cpr * window_ms));
}


int encoder_init(void)
{
	struct encoder *encs[] = { &enc_left, &enc_right };

	for (size_t i = 0; i < ARRAY_SIZE(encs); i++) {
		if (!device_is_ready(encs[i]->device) ||
		    encoder_read_raw(encs[i], &encs[i]->last_raw) != 0) {
			printk("Encoder %s not ready, check overlay\n", encs[i]->name);
			return -ENODEV;
		}
	}
	printk("Encoders ready (cpr L=%d R=%d, period %d ms)\n",
	       (int)enc_left.cpr, (int)enc_right.cpr, ENC_PERIOD_MS);
	return 0;
}

int encoder_sample(int32_t elapsed_ms, struct encoder_sample *sample)
{
	if (elapsed_ms <= 0 || encoder_update(&enc_left) != 0 ||
	    encoder_update(&enc_right) != 0) {
		return -EIO;
	}

	*sample = (struct encoder_sample) {
		.left_position = enc_left.position,
		.right_position = enc_right.position,
		.left_rpm = counts_to_rpm(enc_left.delta, enc_left.cpr, elapsed_ms),
		.right_rpm = counts_to_rpm(enc_right.delta, enc_right.cpr, elapsed_ms),
	};
	return 0;
}
