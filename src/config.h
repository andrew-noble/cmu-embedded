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
#define UART_RX_QUEUE_DEPTH     64
#define UART_RX_QUEUE_ALIGNMENT 1
#define UART_RX_WAIT_MS         5
#define LINK_TIMEOUT_MS         95  /* polled every 5 ms; target 100 ms */
#define PRINT_EVERY             10  /* valid frames */
#define MAIN_PRINT_PERIOD_MS    100

/* Encoder and motor scheduling. */
#define ENC_PERIOD_MS           10
#define MOTOR_STACK_SIZE        2048
#define MOTOR_PRIORITY          0
#define MOTOR_INTEGRAL_LIMIT_RPM_MS 500000LL

/* Blinker timing and thread. Half-periods give 1 Hz turns and 2 Hz hazards. */
#define TURN_HALF_PERIOD_MS     500
#define HAZARD_HALF_PERIOD_MS   250
#define BLINKER_STACK_SIZE      1024
#define BLINKER_PRIORITY        2

/* Current Pi bridge.c documents pedal values as 0..32767 and sends their
 * 16-bit patterns little-endian. Negative readings are rejected until live
 * Pi output resolves the conflict with the reported -32767 release value.
 * Byte 10 is a buttons placeholder (currently zero), not clutch yet.
 */
#define THROTTLE_RAW_MIN           0
#define THROTTLE_RAW_MAX       32767
#define THROTTLE_RAW_REST          0
#define THROTTLE_RAW_FULL      32767
#define THROTTLE_ZERO_PERCENT    1U  /* ignore the first 1% of travel */
#define BRAKE_RAW_MIN              0
#define BRAKE_RAW_MAX          32767
#define BRAKE_PRESSED_AT       16384  /* provisional midpoint */
#define BRAKE_PRESSED_HIGH        1   /* use 0 if values fall when pressed */
#define CLUTCH_RAW_MIN            0U
#define CLUTCH_RAW_MAX          255U
#define CLUTCH_PRESSED_AT       128U
#define CLUTCH_PRESSED_HIGH       1   /* use 0 if values fall when pressed */
#define CLUTCH_CONTROL_ENABLED    0   /* forward only until Pi sends clutch */

/* Allow the controller to command the full PWM range. */
#define MOTOR_OUTPUTS_ENABLED     1
#define MOTOR_MAX_DUTY_PERCENT  100

#define MOTOR_MAX_RPM           120
#define MOTOR_SHIFT_MAX_RPM       5

/* Output duty is a whole percent. PID corrections retain tenths of a percent
 * internally so the initial, untuned gains keep their previous strength. */
#define MOTOR_FF_MAX_PERCENT             40
#define MOTOR_KP_TENTHS_PERCENT_PER_RPM   2
#define MOTOR_KI_TENTHS_PERCENT_PER_RPM_S 1
#define MOTOR_KD_TENTHS_PERCENT_S_PER_RPM 0

/* Set each polarity after a brief, wheel-up test. */
#define LEFT_FORWARD_IN1_HIGH    1
#define RIGHT_FORWARD_IN1_HIGH   1

#endif
