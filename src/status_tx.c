#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "config.h"
#include "crc8.h"
#include "status_tx.h"

/* status_tx thread: every STATUS_PERIOD_MS, send the status frame built in
 * the previous period, then sample the current sensors and build the next.
 * Sending first keeps the send time independent of the ADC read time, so
 * the frame-to-frame interval stays within 20 ms +/- 10%. */

#define STATUS_NODE DT_PATH(zephyr_user)

BUILD_ASSERT(DT_PROP_LEN(STATUS_NODE, io_channels) == 3,
	     "zephyr,user io-channels must list the 3 current sensors");

/* Order matches io-channels in the overlay: left motor, right motor, servo. */
static const struct adc_dt_spec current_adc[] = {
	ADC_DT_SPEC_GET_BY_IDX(STATUS_NODE, 0),
	ADC_DT_SPEC_GET_BY_IDX(STATUS_NODE, 1),
	ADC_DT_SPEC_GET_BY_IDX(STATUS_NODE, 2),
};

/* Byte positions in the status frame (header is bytes 0 to 3). */
enum {
	STATUS_POS_LEFT  = 4,
	STATUS_POS_RIGHT = 6,
	STATUS_POS_SERVO = 8,
	STATUS_POS_STATE = 10,
	STATUS_POS_SEQ   = 11,
	STATUS_POS_CRC   = 12,
};

static atomic_t zone_state = ATOMIC_INIT(ZONE_STATE_ERROR);  /* power-up is an error state */
static bool adc_ok;

#if STATUS_TX_ENABLED
static const struct device *const status_uart = DEVICE_DT_GET(DT_NODELABEL(usart1));
#endif

K_TIMER_DEFINE(status_timer, NULL, NULL);

void status_tx_set_zone_state(enum zone_state state)
{
	atomic_set(&zone_state, state);
}

static int adc_setup(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(current_adc); i++) {
		if (!adc_is_ready_dt(&current_adc[i])) {
			return -ENODEV;
		}
		int rc = adc_channel_setup_dt(&current_adc[i]);

		if (rc < 0) {
			return rc;
		}
	}
	return 0;
}

/* Average STATUS_ADC_SAMPLES conversions of one channel, in millivolts. */
static int read_current_mv(const struct adc_dt_spec *spec, uint16_t *mv_out)
{
	uint16_t raw;
	struct adc_sequence sequence = {
		.buffer = &raw,
		.buffer_size = sizeof(raw),
	};
	int32_t sum = 0;
	int rc = adc_sequence_init_dt(spec, &sequence);

	if (rc < 0) {
		return rc;
	}
	for (int i = 0; i < STATUS_ADC_SAMPLES; i++) {
		rc = adc_read_dt(spec, &sequence);
		if (rc < 0) {
			return rc;
		}
		sum += raw;
	}

	int32_t mv = sum / STATUS_ADC_SAMPLES;

	rc = adc_raw_to_millivolts_dt(spec, &mv);
	if (rc < 0) {
		return rc;
	}
	*mv_out = (uint16_t)CLAMP(mv, 0, STATUS_CURRENT_INVALID - 1);
	return 0;
}

static void build_frame(uint8_t f[FRAME_LEN], uint8_t seq)
{
	uint16_t mv[ARRAY_SIZE(current_adc)];

	for (size_t i = 0; i < ARRAY_SIZE(current_adc); i++) {
		if (!adc_ok || read_current_mv(&current_adc[i], &mv[i]) != 0) {
			mv[i] = STATUS_CURRENT_INVALID;
		}
	}

	f[0] = UART_SOF;
	f[1] = STATUS_ID_HI;
	f[2] = STATUS_ID_LO;
	f[3] = STATUS_DLC;
	sys_put_le16(mv[0], &f[STATUS_POS_LEFT]);
	sys_put_le16(mv[1], &f[STATUS_POS_RIGHT]);
	sys_put_le16(mv[2], &f[STATUS_POS_SERVO]);
	f[STATUS_POS_STATE] = (uint8_t)atomic_get(&zone_state);
	f[STATUS_POS_SEQ] = seq;
	f[STATUS_POS_CRC] = crc8(&f[1], FRAME_LEN - 2);   /* bytes 1 to 11, as the Pi does */
}

static void send_frame(const uint8_t f[FRAME_LEN])
{
#if STATUS_TX_ENABLED
	/* Polling TX: about 1.1 ms busy-wait at 115200 baud. Switch to
	 * interrupt-driven TX once this is verified on the Pi side. */
	for (size_t i = 0; i < FRAME_LEN; i++) {
		uart_poll_out(status_uart, f[i]);
	}
#else
	/* No TX pin yet: print the readings once per STATUS_PRINT_EVERY frames. */
	static uint32_t count;

	if (++count % STATUS_PRINT_EVERY == 0) {
		printk("STATUS mV L %5u R %5u S %5u  state %u  seq %3u\n",
		       sys_get_le16(&f[STATUS_POS_LEFT]),
		       sys_get_le16(&f[STATUS_POS_RIGHT]),
		       sys_get_le16(&f[STATUS_POS_SERVO]),
		       f[STATUS_POS_STATE], f[STATUS_POS_SEQ]);
	}
#endif
}

static void status_tx_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	uint8_t frame[FRAME_LEN];
	uint8_t seq = 0;

	adc_ok = adc_setup() == 0;
	if (!adc_ok) {
		/* Keep sending: the status frame is also the heartbeat. */
		printk("Current sensor ADC not ready, check overlay; reporting invalid readings\n");
	}

	build_frame(frame, seq++);
	k_timer_start(&status_timer, K_MSEC(STATUS_PERIOD_MS), K_MSEC(STATUS_PERIOD_MS));

	while (1) {
		k_timer_status_sync(&status_timer);
		send_frame(frame);
		build_frame(frame, seq++);
	}
}

K_THREAD_DEFINE(status_tx_tid, STATUS_TX_STACK_SIZE, status_tx_entry,
		NULL, NULL, NULL, STATUS_TX_PRIORITY, 0, 0);
