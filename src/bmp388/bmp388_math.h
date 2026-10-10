/*
 * Trim unpacking, compensation and timing, split out so they can be compiled
 * and asserted on the host (see test/test_bmp388_math.c) without an ARM
 * toolchain. Altitude lives in the core: baro_altitude_m().
 *
 * The compensation is the datasheet's floating-point reference (§9), in
 * single precision as printed -- which is also what the Cortex-M4F FPU does in
 * hardware. The datasheet publishes no worked example with numbers in it, so
 * the test checks this against a double-precision transcription of the same
 * listing over a sweep of raw values, and pins a handful of vectors it
 * generated itself.
 *
 * Source: BST-BMP388-DS001-07, revision 1.7 (docs/BMP388.pdf).
 */
#ifndef BMP388_MATH_H
#define BMP388_MATH_H

#include <stdint.h>

/*
 * The NVM trimming coefficients as stored, Table 23: 0x31..0x45, 16-bit words
 * little-endian, signedness per coefficient.
 */
struct bmp388_calib {
	uint16_t t1, t2;
	int8_t t3;
	int16_t p1, p2;
	int8_t p3, p4;
	uint16_t p5, p6;
	int8_t p7, p8;
	int16_t p9;
	int8_t p10, p11;
};

/* The same coefficients scaled to floating point by §9.1: PAR_x. */
struct bmp388_par {
	float t1, t2, t3;
	float p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11;
};

static inline uint16_t bmp388_u16le(const uint8_t *b)
{
	return (uint16_t)(b[0] | (b[1] << 8));
}

/* b[0] is register 0x31. */
static inline void bmp388_parse_calib(const uint8_t b[21],
				      struct bmp388_calib *c)
{
	c->t1 = bmp388_u16le(b + 0);		/* 0x31..0x32 */
	c->t2 = bmp388_u16le(b + 2);		/* 0x33..0x34 */
	c->t3 = (int8_t)b[4];			/* 0x35 */
	c->p1 = (int16_t)bmp388_u16le(b + 5);	/* 0x36..0x37 */
	c->p2 = (int16_t)bmp388_u16le(b + 7);	/* 0x38..0x39 */
	c->p3 = (int8_t)b[9];			/* 0x3A */
	c->p4 = (int8_t)b[10];			/* 0x3B */
	c->p5 = bmp388_u16le(b + 11);		/* 0x3C..0x3D */
	c->p6 = bmp388_u16le(b + 13);		/* 0x3E..0x3F */
	c->p7 = (int8_t)b[15];			/* 0x40 */
	c->p8 = (int8_t)b[16];			/* 0x41 */
	c->p9 = (int16_t)bmp388_u16le(b + 17);	/* 0x42..0x43 */
	c->p10 = (int8_t)b[19];			/* 0x44 */
	c->p11 = (int8_t)b[20];			/* 0x45 */
}

/*
 * NVM_PAR_T1 is the raw-temperature offset (PAR_T1 = T1 * 2^8) and NVM_PAR_P5
 * the pressure offset (PAR_P5 = P5 * 2^3); both unsigned. Neither can be zero
 * on a part that measures anything near room conditions, so zero is a bus
 * that reads nothing. This is a judgement from the formula, not a datasheet
 * rule: unlike the BMP280's, nothing here divides, so a zero trim would not
 * crash -- it would just read nonsense.
 */
static inline int bmp388_calib_ok(const struct bmp388_calib *c)
{
	return c->t1 != 0 && c->p5 != 0;
}

/*
 * §9.1, exactly: every divisor is a power of two, so each PAR_x is the NVM
 * value with its exponent moved -- no rounding, even in single precision.
 * PAR_T1 and PAR_P5 divide by 2^-8 and 2^-3, i.e. multiply.
 */
static inline void bmp388_scale_calib(const struct bmp388_calib *c,
				      struct bmp388_par *par)
{
	par->t1 = (float)c->t1 / 0x1p-8f;
	par->t2 = (float)c->t2 / 0x1p30f;
	par->t3 = (float)c->t3 / 0x1p48f;
	par->p1 = ((float)c->p1 - 0x1p14f) / 0x1p20f;
	par->p2 = ((float)c->p2 - 0x1p14f) / 0x1p29f;
	par->p3 = (float)c->p3 / 0x1p32f;
	par->p4 = (float)c->p4 / 0x1p37f;
	par->p5 = (float)c->p5 / 0x1p-3f;
	par->p6 = (float)c->p6 / 0x1p6f;
	par->p7 = (float)c->p7 / 0x1p8f;
	par->p8 = (float)c->p8 / 0x1p15f;
	par->p9 = (float)c->p9 / 0x1p48f;
	par->p10 = (float)c->p10 / 0x1p48f;
	par->p11 = (float)c->p11 / 0x1p65f;
}

/*
 * Forced measurement time per preset: Table 22's *maximum* column, rounded up
 * to whole ms -- 78.09, 41.89, 21.53, 12.48, 7.96 and 5.70 ms. §3.9.2's
 * formula gives the typical time, so it is not used for the wait. 0 for an
 * osr outside the enum, which is how probe validates one.
 */
static inline int bmp388_meas_ms(int osr)
{
	static const int ms[6] = { 79, 42, 22, 13, 8, 6 };

	return (osr >= 0 && osr <= 5) ? ms[osr] : 0;
}

/*
 * OSR register for a preset, Table 41: osr_t[5:3] | osr_p[2:0], where each
 * field is 0..5 for x1..x32. The presets are Table 6's, with its recommended
 * temperature oversampling, best and slowest first so that a zeroed config
 * asks for the best. The caller has range-checked osr with bmp388_meas_ms();
 * 0x00 is a legal value here (ultra low, x1/x1), so there is no in-band
 * error.
 */
static inline uint8_t bmp388_osr_reg(int osr)
{
	static const uint8_t field[6][2] = {	/* { osr_t, osr_p } */
		{ 1, 5 },	/* highest:    t x2, p x32 */
		{ 1, 4 },	/* ultra high: t x2, p x16 */
		{ 0, 3 },	/* high:       t x1, p x8  */
		{ 0, 2 },	/* standard:   t x1, p x4  */
		{ 0, 1 },	/* low:        t x1, p x2  */
		{ 0, 0 },	/* ultra low:  t x1, p x1  */
	};

	return (uint8_t)((field[osr][0] << 3) | field[osr][1]);
}

/* One 24-bit unsigned result from the burst: xlsb, lsb, msb (Tables 28, 29). */
static inline uint32_t bmp388_raw24(const uint8_t *b)
{
	return (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16);
}

/*
 * TEMPERATURE COMPENSATION, §9.2 BMP388_compensate_temperature: degrees C,
 * which is also t_lin, the value the pressure compensation needs.
 *
 * The listing writes the result into the calibration struct as a side
 * effect; here it is returned and the caller passes it on. The 24-bit raw
 * value converts to float exactly.
 */
static inline float bmp388_compensate_temperature(const struct bmp388_par *par,
						  uint32_t uncomp_temp)
{
	float partial_data1 = (float)uncomp_temp - par->t1;
	float partial_data2 = partial_data1 * par->t2;

	return partial_data2 + (partial_data1 * partial_data1) * par->t3;
}

/*
 * PRESSURE COMPENSATION, §9.3 BMP388_compensate_pressure: pascal. Kept
 * structurally as printed.
 */
static inline float bmp388_compensate_pressure(const struct bmp388_par *par,
					       uint32_t uncomp_press,
					       float t_lin)
{
	float partial_data1, partial_data2, partial_data3, partial_data4;
	float partial_out1, partial_out2;
	float u = (float)uncomp_press;

	partial_data1 = par->p6 * t_lin;
	partial_data2 = par->p7 * (t_lin * t_lin);
	partial_data3 = par->p8 * (t_lin * t_lin * t_lin);
	partial_out1 = par->p5 + partial_data1 + partial_data2 + partial_data3;

	partial_data1 = par->p2 * t_lin;
	partial_data2 = par->p3 * (t_lin * t_lin);
	partial_data3 = par->p4 * (t_lin * t_lin * t_lin);
	partial_out2 = u * (par->p1 + partial_data1 + partial_data2 +
			    partial_data3);

	partial_data1 = u * u;
	partial_data2 = par->p9 + par->p10 * t_lin;
	partial_data3 = partial_data1 * partial_data2;
	partial_data4 = partial_data3 + (u * u * u) * par->p11;

	return partial_out1 + partial_out2 + partial_data4;
}

#endif /* BMP388_MATH_H */
