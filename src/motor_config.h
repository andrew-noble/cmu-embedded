#ifndef MOTOR_CONFIG_H_
#define MOTOR_CONFIG_H_

/* Provisional Pi command contract. Confirm these against bridge.c before driving.
 * Payload bytes: steer int16 LE, throttle uint16 LE, brake uint16 LE,
 * clutch uint8, sequence uint8. CRC and framing are in main.c.
 */
#define THROTTLE_RAW_MIN          0U
#define THROTTLE_RAW_MAX       1000U
#define THROTTLE_RAW_REST         0U
#define THROTTLE_RAW_FULL      1000U
#define THROTTLE_ZERO_PERMILLE  10U  /* ignore the first 1% of travel */
#define BRAKE_RAW_MIN             0U
#define BRAKE_RAW_MAX          1000U
#define BRAKE_PRESSED_AT        500U
#define BRAKE_PRESSED_HIGH        1   /* use 0 if values fall when pressed */
#define CLUTCH_RAW_MIN            0U
#define CLUTCH_RAW_MAX          255U
#define CLUTCH_PRESSED_AT       128U
#define CLUTCH_PRESSED_HIGH       1   /* use 0 if values fall when pressed */

/* Change to 1 only after confirming Pi ranges, clutch behavior, motor polarity,
 * and L298N supply/ground. With 0, firmware still initializes dynamic braking,
 * reads commands and encoders, and computes control, but never drives a motor.
 */
#define MOTOR_OUTPUTS_ENABLED     0

#define MOTOR_MAX_RPM           120
#define MOTOR_SHIFT_MAX_RPM       5

/* Duty is in parts per thousand. Initial gains are starting points, not tuned.
 * P: duty/rpm; I: duty/(rpm*s); D: duty*s/rpm. */
#define MOTOR_FF_MAX_PERMILLE   400
#define MOTOR_KP_PERMILLE_PER_RPM 2
#define MOTOR_KI_PERMILLE_PER_RPM_S 1
#define MOTOR_KD_PERMILLE_S_PER_RPM 0

/* Set each polarity after a brief, wheel-up test. */
#define LEFT_FORWARD_IN1_HIGH    1
#define RIGHT_FORWARD_IN1_HIGH   1

#endif
