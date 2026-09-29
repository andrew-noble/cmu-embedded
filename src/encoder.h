#ifndef ENCODER_H_
#define ENCODER_H_

#include <stdint.h>

struct encoder_sample {
	int32_t left_position;
	int32_t right_position;
	int32_t left_rpm;
	int32_t right_rpm;
};

int encoder_init(void);
int encoder_sample(int32_t elapsed_ms, struct encoder_sample *sample);

#endif
