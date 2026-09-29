#ifndef SERVO_DRIVER_H_
#define SERVO_DRIVER_H_

#include <stdint.h>

/* Initialization leaves PWM off. Pulse widths are microseconds, not duty %. */
int servo_driver_init(void);
int servo_driver_set_pulse(uint32_t pulse_us);
/* Map a validated Pi steering value to the configured endpoints, zero=center. */
int servo_driver_set_steering(int16_t steering);
int servo_driver_disable(void);

#endif
