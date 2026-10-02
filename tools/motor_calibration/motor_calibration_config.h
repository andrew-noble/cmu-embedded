#ifndef CALIBRATION_CONFIG_H_
#define CALIBRATION_CONFIG_H_

/* Settings used only by the separate motor calibration firmware. */
#define CALIBRATION_OUTPUTS_ENABLED   1
#define CALIBRATION_MAX_DUTY_PERCENT 100
#define CAL_COMMAND_TIMEOUT_MS      500
#define CAL_REPORT_PERIOD_MS        100
#define CAL_CONSOLE_POLL_MS           5
#define CAL_INPUT_LINE_BYTES        32
#define CAL_MOTOR_STACK_SIZE      2048
#define CAL_MOTOR_PRIORITY           0

#endif
