#ifndef MOTOR_CONTROL_H_
#define MOTOR_CONTROL_H_

#include <stdbool.h>
#include <stdint.h>

struct motor_command {
	int16_t throttle;
	int16_t brake;
	uint8_t clutch;
	bool valid;
};

struct motor_telemetry {
	int32_t left_position;
	int32_t right_position;
	int32_t left_rpm;
	int32_t right_rpm;
	int32_t target_rpm;
	int32_t duty_percent;
	bool reverse;
};

/* Accept a command only when all pedal values are in range.
 * Returns false and requests braking for invalid or link-down commands. */
bool motor_control_submit(struct motor_command command);
void motor_control_get_telemetry(struct motor_telemetry *telemetry);

#endif
