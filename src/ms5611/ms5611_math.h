/*
 * CRC, compensation and conversion timing, split out so they can be compiled
 * and asserted on the host (see test/test_ms5611_math.c) without an ARM
 * toolchain. Altitude lives in the core: baro_altitude_m().
 *
 * The compensation is integer arithmetic on purpose: the datasheet gives it
 * that way, with a worked example exact to the last digit, so the test checks
 * against the datasheet rather than against itself.
 *
 * The compensation and the conversion times are checked against the
 * MS5611-01BA03 datasheet (06/2017), and the CRC against AN520 rev 004. The
 * section headings cited below are not checked against a sheet.
 */
#ifndef MS5611_MATH_H
#define MS5611_MATH_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Conversion wait per OSR, indexed by enum ms5611_osr (4096 first): the
 * ELECTRICAL CHARACTERISTICS maximum conversion times, 9.04, 4.54, 2.28, 1.17
 * and 0.60 ms, rounded up. There is no data-ready flag, so a fixed wait is
 * all there is. 0 for an osr outside the enum.
 */
static inline int ms5611_conv_ms(int osr)
{
	static const int ms[5] = { 10, 5, 3, 2, 1 };

	return (osr >= 0 && osr <= 4) ? ms[osr] : 0;
}

/* Added to CONV_D1 / CONV_D2 (COMMANDS): 0 at OSR 256, 8 at 4096. */
static inline uint8_t ms5611_osr_cmd(int osr)
{
	return (uint8_t)((4 - osr) * 2);
}

/*
 * CYCLIC REDUNDANCY CHECK, as AN520 gives it: CRC4, polynomial x^4 + x + 1,
 * MSB first over the sixteen PROM bytes, with the low byte of word 7 taken as
 * zero. The CRC itself is bits 3..0 of word 7; bits 7..4 are not covered.
 */
static inline uint8_t ms5611_crc4(const uint16_t prom[8])
{
	uint16_t rem = 0;

	for (int i = 0; i < 16; i++) {
		uint16_t w = prom[i >> 1];

		if (i == 15)
			w = 0;
		rem ^= (i & 1) ? (w & 0x00FF) : (w >> 8);
		for (int b = 0; b < 8; b++)
			rem = (rem & 0x8000) ?
			      (uint16_t)((rem << 1) ^ 0x3000) :
			      (uint16_t)(rem << 1);
	}
	return (uint8_t)((rem >> 12) & 0x0F);
}

/*
 * The CRC matches, and C1..C6 are not zero -- an all-zero PROM has a CRC of
 * zero too, so the CRC alone would accept a bus that reads nothing but 0.
 */
static inline bool ms5611_prom_ok(const uint16_t prom[8])
{
	for (int i = 1; i <= 6; i++)
		if (prom[i] == 0)
			return false;
	return ms5611_crc4(prom) == (prom[7] & 0x000F);
}

/*
 * PRESSURE AND TEMPERATURE CALCULATION, then SECOND ORDER TEMPERATURE
 * COMPENSATION. c[1]..c[6] are C1..C6; d1 is raw pressure, d2 raw
 * temperature. temp comes out in 0.01 degC, pressure in pascal (the
 * datasheet's 0.01 mbar).
 *
 * Divisions truncate toward zero. The datasheet does not say which rounding
 * it means; a right shift floors instead, and differs by 1 LSB -- 0.01 degC
 * or 1 Pa -- on negative intermediates.
 */
static inline void ms5611_compensate(const uint16_t c[8], uint32_t d1,
				     uint32_t d2, int32_t *temp,
				     int32_t *pressure)
{
	int64_t dt = (int64_t)d2 - (int64_t)c[5] * 256;			/* 2^8 */
	int64_t t = 2000 + dt * c[6] / 8388608;				/* 2^23 */
	int64_t off = (int64_t)c[2] * 65536 + (int64_t)c[4] * dt / 128;	/* 2^16, 2^7 */
	int64_t sens = (int64_t)c[1] * 32768 + (int64_t)c[3] * dt / 256;	/* 2^15, 2^8 */

	if (t < 2000) {					/* below 20 degC */
		int64_t t2 = dt * dt / 2147483648LL;			/* 2^31 */
		int64_t off2 = 5 * (t - 2000) * (t - 2000) / 2;
		int64_t sens2 = 5 * (t - 2000) * (t - 2000) / 4;

		if (t < -1500) {			/* below -15 degC */
			off2 += 7 * (t + 1500) * (t + 1500);
			sens2 += 11 * (t + 1500) * (t + 1500) / 2;
		}
		t -= t2;
		off -= off2;
		sens -= sens2;
	}

	*temp = (int32_t)t;
	*pressure = (int32_t)(((int64_t)d1 * sens / 2097152 - off) /
			      32768);				/* 2^21, 2^15 */
}

#endif /* MS5611_MATH_H */
