#ifndef APP_CONFIG_H_
#define APP_CONFIG_H_

/* Shared and normal-firmware settings. Calibration-only settings live under
 * tools/motor_calibration/. Physical pins, PWM periods, and encoder counts
 * per revolution belong in boards/nucleo_f401re.overlay; Zephyr driver options
 * belong in prj.conf. Keep UART frame settings in sync with the Pi sender. */

/* Pi-to-Nucleo UART packet and link supervision. */
#define UART_SOF                0xA5
#define FRAME_LEN               13  /* SOF + ID(2) + DLC + data(8) + CRC */
#define CMD_ID_HI               0x01
#define CMD_ID_LO               0x00
#define CMD_DLC                 8
#define CRC8_POLYNOMIAL         0x07
#define CRC8_INITIAL            0x00
#define UART_RX_QUEUE_DEPTH     1   /* frames, not bytes: keep only the newest */
#define UART_RX_QUEUE_ALIGNMENT 4   /* queue contains frame plus reception timestamp */
/* Requested test policy: three consecutive 150 ms misses, then fail-safe.
 * This is not the handout's literal 100 ms cable-unplug checkoff timing. */
#define LINK_MISS_INTERVAL_MS    150
#define LINK_MISS_LIMIT          3
#define LINK_TIMEOUT_MS          (LINK_MISS_INTERVAL_MS * LINK_MISS_LIMIT)
#define LINK_FAILSAFE_RESPONSE_BUDGET_MS 100 /* measurement budget, not a delay */
#define PRINT_EVERY             10  /* valid frames */
#define MAIN_PRINT_PERIOD_MS    100
#define USB_STATUS_PERIOD_MS    100 /* continuous summary, even with Pi link down */

/* Encoder and motor scheduling. */
#define ENC_PERIOD_MS           10
/* Allow the integral alone to reach full effort; keep its storage bounded.
 * Derive the bound from Ki so lowering Ki cannot silently lower max PWM. */
#define MOTOR_INTEGRAL_LIMIT_RPM_MS \
	(MOTOR_MAX_DUTY_PERCENT * 1000000LL / \
	 (MOTOR_KI_MILLI_PERCENT_PER_RPM_S > 0 ? MOTOR_KI_MILLI_PERCENT_PER_RPM_S : 1))

/* Pi byte 10 button bits from the steering wheel, 1 while held. Byte 10 has
 * only 8 bits, so the Pi packs wheel button indices 10/5/4 into bits 0/1/2.
 * A press is a 0 -> 1 change between valid frames.
 * Self-test: one press enters hazards + dynamic braking; two presses within
 * SELF_TEST_EXIT_WINDOW_MS (after entering) exit. Left/right toggle turns. */
#define BUTTON_SELF_TEST        (1U << 0)   /* wheel button index 10 */
#define BUTTON_LEFT             (1U << 1)   /* wheel button index 5 */
#define BUTTON_RIGHT            (1U << 2)   /* wheel button index 4 */
#define SELF_TEST_EXIT_WINDOW_MS 500
#define BUTTON_DEBOUNCE_MS       30 /* observed release interval before another press */

/* Blinker timing and thread. Half-periods give 1 Hz turns and 2 Hz hazards. */
#define TURN_HALF_PERIOD_MS     500
#define HAZARD_HALF_PERIOD_MS   250
/* Arm 500 counts beyond neutral; cancel on return to the dead zone. */
#define BLINKER_TURN_MARGIN     500
#define BLINKER_TURN_THRESHOLD (STEER_DEAD_ZONE + BLINKER_TURN_MARGIN)

/* Thread priorities and stacks: the Part 4 task table.
 * Zephyr: lower number = higher priority; all are preemptive.
 * The motor thread sits above cmd_handler so a submitted brake or throttle
 * command is applied before cmd_handler continues (2 ms path, R2.1/R2.2). */
#define MOTOR_PRIORITY          0   /* 10 ms PID + immediate step per command */
#define FAIL_SAFE_PRIORITY      1   /* wakes only on link_timer expiry */
#define CMD_HANDLER_PRIORITY    2   /* one run per valid frame from the Pi */
#define STATUS_TX_PRIORITY      3   /* 20 ms status frame */
#define BLINKER_PRIORITY        4   /* partner's blinker thread */
#define MOTOR_STACK_SIZE        2048
#define FAIL_SAFE_STACK_SIZE    1024
#define CMD_HANDLER_STACK_SIZE  2048
#define STATUS_TX_STACK_SIZE    1024
#define BLINKER_STACK_SIZE      1024
#define CMD_HANDLER_START_DELAY_MS 0

/* STM32-to-Pi status frame: same 13-byte layout as the command frame, with
 * ID 0x200. Data = three current-sensor readings (mV, uint16 LE), zone state,
 * sequence counter. Readings are millivolts until the sensor scale is known. */
#define STATUS_PERIOD_MS        20  /* R4.5 substitute: 20 ms +/- 10% */
#define STATUS_ID_HI            0x02
#define STATUS_ID_LO            0x00
#define STATUS_DLC              8
#define STATUS_ADC_SAMPLES      8   /* averaged per channel per frame */
#define STATUS_CURRENT_INVALID  0xFFFF
/* Nominal ACS712-05B, 5 V supply, direct output (no divider).
 * Assumes all three channels use this variant. Replace zero offsets with
 * measured no-current voltages; these constants do not change ADC limits. */
#define CURRENT_LEFT_ZERO_MV       2500
#define CURRENT_RIGHT_ZERO_MV      2500
#define CURRENT_SERVO_ZERO_MV      2500
#define CURRENT_LEFT_MV_PER_AMP     185
#define CURRENT_RIGHT_MV_PER_AMP    185
#define CURRENT_SERVO_MV_PER_AMP    185
/* 0 until USART1 has a TX pin: PB6 drives right motor IN3 and PA9 is the
 * right encoder. While 0, frames are built but not sent, and readings are
 * printed once per STATUS_PRINT_EVERY frames instead. */
#define STATUS_TX_ENABLED       1
#define STATUS_PRINT_EVERY      50

/* Pi pedal fields are 16-bit little-endian values. Throttle uses 0..32767;
 * measured brake endpoints are 30454 released and 32767 fully pressed.
 * Negative readings are rejected.
 * Byte 10 carries the wheel buttons (BUTTON_* above), not clutch.
 */
#define STEER_RAW_MIN         -32767
#define STEER_RAW_MAX          32766
#define STEER_DEAD_ZONE          500 /* inclusive -500..500 commands center */
/* LD-1501MG: negative steering -> LEFT, positive -> RIGHT.
 * Swap LEFT/RIGHT pulse widths if the linkage direction is reversed. */
#define SERVO_PULSE_MIN_US       500
#define SERVO_PULSE_MAX_US      2500
#define SERVO_LEFT_US          1000
#define SERVO_CENTER_US        1500
#define SERVO_RIGHT_US         2000
#define THROTTLE_RAW_MIN           0
#define THROTTLE_RAW_MAX       32767
#define THROTTLE_RAW_REST          0
#define THROTTLE_RAW_FULL      32767
#define THROTTLE_ZERO_PERCENT    1U  /* ignore the first 1% of travel */
#define BRAKE_RAW_MIN              0  /* valid Pi range; rest may drift below 30454 */
#define BRAKE_RAW_MAX          32767
#define BRAKE_RAW_REST         30454  /* measured reference, not a validity limit */
#define BRAKE_PRESSED_AT       31610  /* midpoint of measured pedal range */
#define BRAKE_PRESSED_HIGH        1   /* use 0 if values fall when pressed */
#define CLUTCH_RAW_MIN            0U
#define CLUTCH_RAW_MAX          255U
#define CLUTCH_PRESSED_AT       128U
#define CLUTCH_PRESSED_HIGH       1   /* use 0 if values fall when pressed */
#define CLUTCH_CONTROL_ENABLED    0   /* forward only until Pi sends clutch */

/* Operational input clamps, independent of RAW validity/calibration ranges.
 * Example: THROTTLE_CLAMP_MAX=10000 limits the target on the ORIGINAL scale.
 * Clamped commands remain valid; telemetry reports the received/applied values.
 * Brake clamping also changes what reaches BRAKE_PRESSED_AT. */
#define STEER_CLAMP_MIN          STEER_RAW_MIN
#define STEER_CLAMP_MAX          STEER_RAW_MAX
#define THROTTLE_CLAMP_MIN       THROTTLE_RAW_MIN /* must include released pedal */
#define THROTTLE_CLAMP_MAX       THROTTLE_RAW_MAX 
#define BRAKE_CLAMP_MIN          BRAKE_RAW_MIN
#define BRAKE_CLAMP_MAX          BRAKE_RAW_MAX
#define INPUT_CLAMP_REPORT_MS    USB_STATUS_PERIOD_MS

/* Allow the controller to command the full PWM range. */
#define MOTOR_OUTPUTS_ENABLED     1
#define MOTOR_MAX_DUTY_PERCENT  100
/* Normal controller only: positive output maps into 20..100% actual PWM.
 * Zero still stops; calibration commands remain direct duty percentages. */
#define MOTOR_DUTY_OFFSET_PERCENT 20

/* Wheel-up 100% PWM average, corrected from 3960 to 1316 counts/revolution:
 * ((106.33 + 104.80) / 2) * 3960 / 1316 = approximately 317 RPM.
 * This is the estimated unloaded hardware maximum, not the control target.
 * Limit the requested speed to 200 RPM to leave headroom for load correction;
 * PID remains active at full throttle and may command up to 100% PWM. */
#define MOTOR_MAX_RPM           200
#define MOTOR_SHIFT_MAX_RPM       5

/* Output duty is a whole percent; feedback retains thousandths internally. */
/* Gain units: thousandths of effort-percent (150 means 0.15). */
#define MOTOR_KP_MILLI_PERCENT_PER_RPM   450
#define MOTOR_KI_MILLI_PERCENT_PER_RPM_S 600
#define MOTOR_KD_MILLI_PERCENT_S_PER_RPM 20
#define MOTOR_D_FILTER_MS               50

/* Set each polarity after a brief, wheel-up test. */
#define LEFT_FORWARD_IN1_HIGH    1
#define RIGHT_FORWARD_IN1_HIGH   1

#endif
