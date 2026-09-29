#ifndef CRC8_H_
#define CRC8_H_

#include <stddef.h>
#include <stdint.h>

#include "config.h"

/* CRC-8 (poly CRC8_POLYNOMIAL, init CRC8_INITIAL), identical to the Pi's.
 * Shared by the command parser and the status frame builder. */
static inline uint8_t crc8(const uint8_t *buf, size_t len)
{
	uint8_t crc = CRC8_INITIAL;

	for (size_t i = 0; i < len; i++) {
		crc ^= buf[i];
		for (int b = 0; b < 8; b++) {
			crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ CRC8_POLYNOMIAL)
					   : (uint8_t)(crc << 1);
		}
	}
	return crc;
}

#endif
