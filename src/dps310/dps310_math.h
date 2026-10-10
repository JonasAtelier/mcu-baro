/*
 * Calibration unpacking and the compensation polynomial, split out so they
 * can be compiled and asserted on the host (see test/test_dps310_math.c)
 * without an ARM toolchain. Altitude lives in the core: baro_altitude_m().
 *
 * This is the part of a DPS310 driver that is actually hard. The chip hands
 * you nine factory coefficients packed into eighteen bytes at odd bit widths,
 * and a raw reading means nothing until it has been through a two-variable
 * polynomial with them. Get one sign extension wrong and the pressure is
 * plausible, stable, and out by tens of hectopascal.
 */
#ifndef DPS310_MATH_H
#define DPS310_MATH_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Scaling divisors per oversampling code, from the datasheet. Both reference
 * drivers carry this identical table.
 */
static inline float dps310_scale_factor(int prc)
{
	static const float k[8] = {
		 524288.0f,	/* 1x   */
		1572864.0f,	/* 2x   */
		3670016.0f,	/* 4x   */
		7864320.0f,	/* 8x   */
		 253952.0f,	/* 16x  */
		 516096.0f,	/* 32x  */
		1040384.0f,	/* 64x  */
		2088960.0f,	/* 128x */
	};

	return (prc >= 0 && prc <= 7) ? k[prc] : 0.0f;
}

/* Oversampling ratio for a code, for reporting. */
static inline int dps310_oversampling(int prc)
{
	return (prc >= 0 && prc <= 7) ? (1 << prc) : 0;
}

/* Measurements per second for a rate code. */
static inline int dps310_rate_hz(int rate)
{
	return (rate >= 0 && rate <= 7) ? (1 << rate) : 0;
}

/*
 * Background mode has one second to fit every measurement it was asked for,
 * pressure and temperature together; Infineon's own driver refuses to start
 * one that does not fit (DPS__FAIL_UNFINISHED). One measurement takes 2.0 ms
 * plus 1.6 ms per oversample -- 3.6, 5.2, 8.4, 14.8, 27.6, 53.2, 104.4,
 * 206.8 ms for 1x..128x.
 *
 * Formula and limit from Infineon's arduino-xensiv-dps3xx:
 * src/DpsClass.cpp calcBusyTime() ("formula from datasheet"), and
 * src/util/dps_config.h DPS3xx__MAX_BUSYTIME, which keeps 10 ms back.
 * Both in tenths of a millisecond; the datasheet PDF is not in docs/.
 */
#define DPS310_BUSY_LIMIT	9900	/* 990 ms; the total must stay below it */

static inline int dps310_busy_time(int rate, int prc)
{
	return dps310_rate_hz(rate) * (20 + 16 * dps310_oversampling(prc));
}

/*
 * Above 8x oversampling the result is shifted, and CFG_REG must be told so.
 * Forgetting this is a factor-of-many error that looks like a broken sensor.
 */
static inline bool dps310_needs_shift(int prc)
{
	return prc > 3;
}

/* Two's complement sign extension of the low `bits` of v. */
static inline int32_t dps310_sext(uint32_t v, int bits)
{
	uint32_t sign = 1u << (bits - 1);
	uint32_t mask = (bits >= 32) ? 0xFFFFFFFFu : ((1u << bits) - 1);

	v &= mask;
	return (int32_t)((v ^ sign) - sign);
}

/* The nine factory coefficients. */
struct dps310_coefs {
	int32_t c0, c1;				/* 12-bit */
	int32_t c00, c10;			/* 20-bit */
	int32_t c01, c11, c20, c21, c30;	/* 16-bit */
};

/*
 * Unpack the eighteen COEF bytes. The widths are 12, 12, 20, 20 then 16 --
 * they do not line up with byte boundaries and three of them straddle a
 * nibble, which is where a hand-written unpack goes wrong.
 */
static inline void dps310_unpack_coefs(const uint8_t *c,
				       struct dps310_coefs *out)
{
	out->c0 = dps310_sext(((uint32_t)c[0] << 4) | (c[1] >> 4), 12);
	out->c1 = dps310_sext(((uint32_t)(c[1] & 0x0F) << 8) | c[2], 12);
	out->c00 = dps310_sext(((uint32_t)c[3] << 12) | ((uint32_t)c[4] << 4) |
			       (c[5] >> 4), 20);
	out->c10 = dps310_sext(((uint32_t)(c[5] & 0x0F) << 16) |
			       ((uint32_t)c[6] << 8) | c[7], 20);
	out->c01 = dps310_sext(((uint32_t)c[8] << 8) | c[9], 16);
	out->c11 = dps310_sext(((uint32_t)c[10] << 8) | c[11], 16);
	out->c20 = dps310_sext(((uint32_t)c[12] << 8) | c[13], 16);
	out->c21 = dps310_sext(((uint32_t)c[14] << 8) | c[15], 16);
	out->c30 = dps310_sext(((uint32_t)c[16] << 8) | c[17], 16);
}

/* A result register is 24-bit two's complement, most significant byte first. */
static inline int32_t dps310_raw24(const uint8_t *p)
{
	uint32_t v = ((uint32_t)p[0] << 16) | ((uint32_t)p[1] << 8) | p[2];

	return dps310_sext(v, 24);
}

/* Scaled temperature, the shared term of both compensations. */
static inline float dps310_traw_sc(int32_t traw, float kt)
{
	return kt > 0.0f ? (float)traw / kt : 0.0f;
}

/* T = c0 * 0.5 + c1 * Traw_sc */
static inline float dps310_temperature_c(const struct dps310_coefs *k,
					 float traw_sc)
{
	return (float)k->c0 * 0.5f + (float)k->c1 * traw_sc;
}

/*
 * P = c00 + Praw_sc*(c10 + Praw_sc*(c20 + Praw_sc*c30))
 *         + Traw_sc*c01 + Traw_sc*Praw_sc*(c11 + Praw_sc*c21)
 *
 * Result is pascal. Note the pressure depends on the temperature term, so a
 * wrong temperature quietly corrupts the pressure too -- which is why the
 * COEF_SRCE mismatch in the driver matters as much as it does.
 */
static inline float dps310_pressure_pa(const struct dps310_coefs *k,
				       float praw_sc, float traw_sc)
{
	return (float)k->c00 +
	       praw_sc * ((float)k->c10 +
			  praw_sc * ((float)k->c20 + praw_sc * (float)k->c30)) +
	       traw_sc * (float)k->c01 +
	       traw_sc * praw_sc * ((float)k->c11 + praw_sc * (float)k->c21);
}

#endif /* DPS310_MATH_H */
