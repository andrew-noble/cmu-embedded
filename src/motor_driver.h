#ifndef MOTOR_DRIVER_H_
#define MOTOR_DRIVER_H_

#include <stdbool.h>
#include <stdint.h>

enum motor_output_state {
	MOTOR_OUTPUT_UNKNOWN,
	MOTOR_OUTPUT_COAST,
	MOTOR_OUTPUT_BRAKE,
	MOTOR_OUTPUT_DRIVE,
	MOTOR_OUTPUT_FAULT,
};

/* Last software output result, not electrical feedback from the H-bridge. */
enum motor_output_state motor_driver_get_state(void);

/* Shared L298N output operations used by normal and calibration firmware. */
bool motor_driver_init(void);
int motor_driver_brake(void);
/* Disable both bridge enables so the wheels can coast. */
int motor_driver_coast(void);
int motor_driver_drive(bool reverse, int32_t duty_percent);
int motor_driver_drive_wheels(bool reverse, int32_t left_duty, int32_t right_duty);

#endif
