#ifndef TESTPOINT_H_
#define TESTPOINT_H_

/* Scope test points from the Lab 2 test-point board. Each one is optional:
 * until its pin is assigned in the overlay, toggling it does nothing. */
enum testpoint {
	TP_CMD_RX,    /* toggles when a valid Pi command is received */
	TP_PWM_SET,   /* toggles right after a command-triggered motor duty write */
	TP_COUNT,
};

int testpoint_init(void);
void testpoint_toggle(enum testpoint tp);

#endif
