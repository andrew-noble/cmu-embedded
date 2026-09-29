#ifndef MOTOR_DRIVER_H_
#define MOTOR_DRIVER_H_

#include <stdbool.h>
#include <stdint.h>

/* Shared L298N output operations used by normal and calibration firmware. */
bool motor_driver_init(void);
int motor_driver_brake(void);
/* Disable both bridge enables so the wheels can coast. */
int motor_driver_coast(void);
int motor_driver_drive(bool reverse, int32_t duty_percent);

#endif
