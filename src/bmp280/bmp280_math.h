/*
 * Trim unpacking, compensation and timing, split out so they can be compiled
 * and asserted on the host (see test/test_bmp280_math.c) without an ARM
 * toolchain. Altitude lives in the core: baro_altitude_m().
 *
 * The compensation is the datasheet's integer reference code: int32 for
 * temperature, int64 for pressure. The datasheet publishes no worked example
 * with numbers in it, so the test checks this against the datasheet's *other*
 * reference implementation -- the double-precision one -- over a sweep of raw
 * values, and pins a handful of vectors it generated itself.
 *
 * All of it is from memory of the BMP280 datasheet. Nothing here has been
 * checked against a PDF or a physical chip.
 */
#ifndef BMP280_MATH_H
#define BMP280_MATH_H

#include <stdbool.h>
#include <stdint.h>

/*
 * CALIBRATION DATA: 0x88..0x9F, dig_T1 unsigned, the rest signed, all
 * little-endian.
 */
struct bmp280_calib {
	uint16_t t1;
	int16_t t2, t3;
	uint16_t p1;
	int16_t p2, p3, p4, p5, p6, p7, p8, p9;
};

static inline uint16_t bmp280_u16le(const uint8_t *b)
{
	return (uint16_t)(b[0] | (b[1] << 8));
}

static inline void bmp280_parse_calib(const uint8_t b[24],
				      struct bmp280_calib *c)
{
	c->t1 = bmp280_u16le(b + 0);
	c->t2 = (int16_t)bmp280_u16le(b + 2);
	c->t3 = (int16_t)bmp280_u16le(b + 4);
	c->p1 = bmp280_u16le(b + 6);
	c->p2 = (int16_t)bmp280_u16le(b + 8);
	c->p3 = (int16_t)bmp280_u16le(b + 10);
	c->p4 = (int16_t)bmp280_u16le(b + 12);
	c->p5 = (int16_t)bmp280_u16le(b + 14);
	c->p6 = (int16_t)bmp280_u16le(b + 16);
	c->p7 = (int16_t)bmp280_u16le(b + 18);
	c->p8 = (int16_t)bmp280_u16le(b + 20);
	c->p9 = (int16_t)bmp280_u16le(b + 22);
}

/*
 * dig_T1 and dig_P1 are unsigned and never zero on a real part. Zero means a
 * bus that reads nothing, or a trim image that has not been copied out of NVM
 * yet -- and a zero dig_P1 divides by zero further down.
 */
static inline bool bmp280_calib_ok(const struct bmp280_calib *c)
{
	return c->t1 != 0 && c->p1 != 0;
}

/*
 * ctrl_meas for an oversampling preset: osrs_t[7:5] | osrs_p[4:2] | mode[1:0],
 * where the osrs fields are 1..5 for x1..x16. The presets are the datasheet's
 * recommended pairs, indexed by enum bmp280_osr (ultra high resolution first).
 * 0 for an osr outside the enum, which is not a legal ctrl_meas.
 */
static inline uint8_t bmp280_ctrl_meas(int osr, uint8_t mode)
{
	static const uint8_t field[5][2] = {	/* { osrs_t, osrs_p } */
		{ 2, 5 },	/* ultra high: t x2, p x16 */
		{ 1, 4 },	/* high:       t x1, p x8  */
		{ 1, 3 },	/* standard:   t x1, p x4  */
		{ 1, 2 },	/* low:        t x1, p x2  */
		{ 1, 1 },	/* ultra low:  t x1, p x1  */
	};

	if (osr < 0 || osr > 4)
		return 0;
	return (uint8_t)((field[osr][0] << 5) | (field[osr][1] << 2) |
			 (mode & 0x03));
}

/*
 * Forced measurement time per preset, the datasheet maximum rounded up:
 *
 *   t_max = 1.25 + 2.3 * osrs_t + (2.3 * osrs_p + 0.575)   ms
 *
 * with osrs_* the oversampling multipliers, giving 43.2, 22.5, 13.3, 8.7 and
 * 6.4 ms. Forced mode has a measuring flag, but polling it costs a transfer
 * per attempt and the fixed wait is what the flag would end up waiting for
 * anyway. 0 for an osr outside the enum.
 */
static inline int bmp280_meas_ms(int osr)
{
	static const int ms[5] = { 44, 23, 14, 9, 7 };

	return (osr >= 0 && osr <= 4) ? ms[osr] : 0;
}

/*
 * TEMPERATURE COMPENSATION, the datasheet's bmp280_compensate_T_int32: 0.01
 * degC out, plus t_fine, which the pressure compensation needs.
 *
 * var1 and var2 are int64 here where the datasheet has int32. In range the
 * values are identical; out of range -- a garbled raw value against an
 * unlucky trim word -- the int32 version overflows, and signed overflow is
 * undefined rather than merely wrong.
 */
static inline int32_t bmp280_compensate_T_int32(const struct bmp280_calib *c,
						int32_t adc_T, int32_t *t_fine)
{
	int64_t d = (adc_T >> 4) - (int64_t)c->t1;
	int64_t var1 = (((adc_T >> 3) - ((int64_t)c->t1 << 1)) * c->t2) >> 11;
	int64_t var2 = (((d * d) >> 12) * c->t3) >> 14;

	*t_fine = (int32_t)(var1 + var2);
	return (*t_fine * 5 + 128) >> 8;
}

/*
 * PRESSURE COMPENSATION, the datasheet's bmp280_compensate_P_int64: pressure
 * in Q24.8 pascal, so 24674867 is 96386.2 Pa.
 *
 * The shifts of the datasheet listing are written as multiplications wherever
 * the operand can be negative -- left-shifting a negative value is undefined
 * in C11, and var1 goes negative below roughly 25 degC. The right shifts are
 * left alone: they floor, which is what the datasheet means.
 *
 * Returns 0 when var1 is 0, as the datasheet's own listing does. That is the
 * divide-by-zero guard: dig_P1 = 0 lands there, and so can a pathological
 * trim set. It also returns 0 where the datasheet returns nonsense: the final
 * value is signed, and a raw reading far outside the part's 300-1100 hPa range
 * makes it negative, which the datasheet's cast to uint32_t turns into about
 * 16 MPa. The driver's read turns either 0 into -ERANGE.
 */
static inline uint32_t bmp280_compensate_P_int64(const struct bmp280_calib *c,
						 int32_t adc_P, int32_t t_fine)
{
	int64_t var1, var2, p;

	var1 = (int64_t)t_fine - 128000;
	var2 = var1 * var1 * (int64_t)c->p6;
	var2 = var2 + var1 * (int64_t)c->p5 * 131072;		/* << 17 */
	var2 = var2 + (int64_t)c->p4 * 34359738368LL;		/* << 35 */
	var1 = ((var1 * var1 * (int64_t)c->p3) >> 8) +
	       var1 * (int64_t)c->p2 * 4096;
	var1 = ((140737488355328LL + var1) * (int64_t)c->p1) >> 33; /* 1<<47 */
	if (var1 == 0)
		return 0;

	p = 1048576 - adc_P;
	p = ((p * 2147483648LL - var2) * 3125) / var1;		/* << 31 */
	var1 = ((int64_t)c->p9 * (p >> 13) * (p >> 13)) >> 25;
	var2 = ((int64_t)c->p8 * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (int64_t)c->p7 * 16;
	return p > 0 ? (uint32_t)p : 0;
}

/* One 20-bit result from the burst: msb, lsb, then xlsb in bits 7:4. */
static inline int32_t bmp280_raw20(const uint8_t *b)
{
	return (int32_t)(((uint32_t)b[0] << 12) | ((uint32_t)b[1] << 4) |
			 (b[2] >> 4));
}

#endif /* BMP280_MATH_H */
