/*
 * Host-side check of the trim unpacking, the compensation and the timing
 * tables. Altitude is the core's and is checked in test_core.c.
 *
 * The BMP280 datasheet publishes no worked example -- no calibration set with
 * raw values and the answers next to them -- so there is nothing to reproduce
 * exactly, and nothing here claims to. What it does publish is two
 * independent reference implementations of the same arithmetic: the integer
 * one the driver uses, and a double-precision one. test_agrees_with_double()
 * runs both over a sweep of raw values and asserts they land in the same
 * place.
 *
 * The fixed vectors in test_pinned_vectors() are SELF-GENERATED: this
 * implementation's own output, captured so that a later edit that changes a
 * number has to say so. They are not datasheet values and are not evidence
 * that the compensation is right, only that it has not moved.
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bmp280.h"
#include "bmp280_math.h"
#include "bmp280_regs.h"

/*
 * A plausible trim set: the magnitudes and signs a real BMP280 carries, made
 * up for the test. There is no published BMP280 vector to use instead.
 */
static const struct bmp280_calib CAL = { 27504, 26435, -1000, 36477, -10685,
					 3024, 2855, 140, -7, 15500, -14600,
					 6000 };

/* ------------------------------------------ the datasheet's double code ---- */

/*
 * bmp280_compensate_T_double and bmp280_compensate_P_double, the
 * floating-point reference implementation from the datasheet, transcribed
 * from memory and kept structurally as printed. t_fine is a global there too.
 */
static int32_t g_t_fine;

static double compensate_T_double(const struct bmp280_calib *c, int32_t adc_T)
{
	double var1, var2;

	var1 = (((double)adc_T) / 16384.0 - ((double)c->t1) / 1024.0) *
	       ((double)c->t2);
	var2 = ((((double)adc_T) / 131072.0 - ((double)c->t1) / 8192.0) *
		(((double)adc_T) / 131072.0 - ((double)c->t1) / 8192.0)) *
	       ((double)c->t3);
	g_t_fine = (int32_t)(var1 + var2);
	return (var1 + var2) / 5120.0;
}

static double compensate_P_double(const struct bmp280_calib *c, int32_t adc_P)
{
	double var1, var2, p;

	var1 = ((double)g_t_fine / 2.0) - 64000.0;
	var2 = var1 * var1 * ((double)c->p6) / 32768.0;
	var2 = var2 + var1 * ((double)c->p5) * 2.0;
	var2 = (var2 / 4.0) + (((double)c->p4) * 65536.0);
	var1 = (((double)c->p3) * var1 * var1 / 524288.0 +
		((double)c->p2) * var1) / 524288.0;
	var1 = (1.0 + var1 / 32768.0) * ((double)c->p1);
	if (var1 == 0.0)
		return 0.0;		/* the datasheet's own guard */
	p = 1048576.0 - (double)adc_P;
	p = (p - (var2 / 4096.0)) * 6250.0 / var1;
	var1 = ((double)c->p9) * p * p / 2147483648.0;
	var2 = p * ((double)c->p8) / 32768.0;
	return p + (var1 + var2 + ((double)c->p7)) / 16.0;
}

/* -------------------------------------------------------------- tests ---- */

/*
 * The integer path against the double path, over raw values spanning far more
 * than the part's real range (300-1100 hPa, -40 to +85 C). Steps are
 * prime-ish so the sweep does not settle on round numbers.
 *
 * The bounds are what this sweep actually produces, rounded up: 0.0069 C,
 * 0.54 Pa and 13 counts of t_fine. The two implementations round differently
 * -- they are not expected to be identical, only to agree far below the
 * sensor's own noise, which is around 1.3 Pa RMS at the highest oversampling.
 */
static void test_agrees_with_double(void)
{
	double worst_t = 0.0, worst_p = 0.0;
	long worst_tf = 0;
	int checked = 0;

	for (int32_t at = 250000; at <= 800000; at += 3571) {
		int32_t t_fine;
		int32_t ti = bmp280_compensate_T_int32(&CAL, at, &t_fine);
		double td = compensate_T_double(&CAL, at);
		double dt = fabs((double)ti / 100.0 - td);

		if (labs((long)t_fine - (long)g_t_fine) > worst_tf)
			worst_tf = labs((long)t_fine - (long)g_t_fine);
		if (dt > worst_t)
			worst_t = dt;

		for (int32_t ap = 200000; ap <= 900000; ap += 7919) {
			uint32_t pi = bmp280_compensate_P_int64(&CAL, ap,
								t_fine);
			double pd = compensate_P_double(&CAL, ap);
			double dp = fabs((double)pi / 256.0 - pd);

			if (pd <= 0.0)
				continue;	/* the guarded branch; below */
			if (dp > worst_p)
				worst_p = dp;
			checked++;
		}
	}

	assert(checked > 10000);
	assert(worst_t <= 0.01 && "integer and double temperature must agree");
	assert(worst_tf <= 32 && "and on t_fine, 5120 counts per degree");
	assert(worst_p <= 1.0 && "integer and double pressure must agree");
}

/*
 * Fixed vectors, SELF-GENERATED from this implementation -- not datasheet
 * numbers. They exist so that a change to the arithmetic has to be
 * deliberate. The raws are off the sweep grid, so each one is cross-checked
 * against the double implementation here as well -- pinning a wrong number
 * is no use.
 */
static void test_pinned_vectors(void)
{
	static const struct {
		int32_t adc_t, adc_p, t_fine, temp_c100;
		uint32_t press_q24_8;
	} v[] = {
		{ 519888, 415148,  128422,  2508, 25767233 },	/*  25.08 C, 100653.3 Pa */
		{ 415148, 519888,  -40245,  -786, 20115057 },	/*  -7.86 C,  78574.4 Pa */
		{ 300000, 250000, -227131, -4436, 29615363 },	/* -44.36 C, 115685.0 Pa */
		{ 700000, 850000,  415464,  8115,  7287447 },	/*  81.15 C,  28466.6 Pa */
		{ 524288, 524288,  135479,  2646, 21003398 },	/*  26.46 C,  82044.5 Pa */
	};

	for (unsigned i = 0; i < sizeof(v) / sizeof(v[0]); i++) {
		int32_t t_fine;
		int32_t temp = bmp280_compensate_T_int32(&CAL, v[i].adc_t,
							 &t_fine);

		assert(t_fine == v[i].t_fine);
		assert(temp == v[i].temp_c100);
		assert(bmp280_compensate_P_int64(&CAL, v[i].adc_p, t_fine) ==
		       v[i].press_q24_8);

		compensate_T_double(&CAL, v[i].adc_t);	/* sets g_t_fine */
		assert(fabs(compensate_P_double(&CAL, v[i].adc_p) -
			    (double)v[i].press_q24_8 / 256.0) < 1.0);
	}
}

/*
 * The corners of the 20-bit raw range, on the test trim. None of this has a
 * physical meaning -- it is the shape of input a broken bus produces -- so the
 * claims are narrow: the guard catches everything the double implementation
 * calls negative, the two still agree where it does not (worst 0.53 Pa), and
 * the arithmetic stays inside int64_t -- build this file with
 * -fsanitize=undefined and the run is silent, which is what backs the overflow
 * note in the README.
 */
static void test_corners(void)
{
	static const int32_t corner[4] = { 0, 1, 0x80000, 0xFFFFF };

	for (int i = 0; i < 4; i++) {
		int32_t t_fine;

		bmp280_compensate_T_int32(&CAL, corner[i], &t_fine);
		compensate_T_double(&CAL, corner[i]);	/* sets g_t_fine */
		for (int j = 0; j < 4; j++) {
			uint32_t pi = bmp280_compensate_P_int64(&CAL, corner[j],
								t_fine);
			double pd = compensate_P_double(&CAL, corner[j]);

			if (pd > 0.0)
				assert(fabs((double)pi / 256.0 - pd) < 1.0);
			else
				assert(pi == 0 &&
				       "the guard, not a wrapped uint32_t");
		}
	}
}

static void test_divide_by_zero_and_wraparound(void)
{
	struct bmp280_calib c = CAL;
	int32_t t_fine;

	/*
	 * dig_P1 = 0 makes var1 zero, and the datasheet's guard returns 0
	 * rather than dividing by it. That is the crash this is here to
	 * prevent.
	 */
	c.p1 = 0;
	bmp280_compensate_T_int32(&c, 519888, &t_fine);
	assert(bmp280_compensate_P_int64(&c, 415148, t_fine) == 0);
	assert(!bmp280_calib_ok(&c) && "probe refuses that trim set anyway");

	/* An all-zero trim image, which is what a dead bus reads. */
	memset(&c, 0, sizeof(c));
	bmp280_compensate_T_int32(&c, 519888, &t_fine);
	assert(bmp280_compensate_P_int64(&c, 415148, t_fine) == 0);
	assert(!bmp280_calib_ok(&c));

	/*
	 * Raw pressure at the top of its 20-bit range is far below the part's
	 * range and comes out negative. The datasheet casts that to uint32_t
	 * and reports about 16 MPa; this returns 0, which the driver reports as
	 * an error.
	 */
	bmp280_compensate_T_int32(&CAL, 519888, &t_fine);
	assert(compensate_P_double(&CAL, 1048575) < 0.0);
	assert(bmp280_compensate_P_int64(&CAL, 1048575, t_fine) == 0);

	/* Just inside the real range it is an ordinary number again. */
	assert(bmp280_compensate_P_int64(&CAL, 415148, t_fine) > 0);
}

static void test_parse_calib(void)
{
	/*
	 * dig_T1 and dig_P1 unsigned, the other ten signed, all little-endian.
	 * 0xFFFF as dig_T1 is 65535; as dig_T2 it is -1.
	 */
	uint8_t b[BMP280_CALIB_LEN];
	struct bmp280_calib c;

	memset(b, 0xFF, sizeof(b));
	bmp280_parse_calib(b, &c);
	assert(c.t1 == 65535 && c.p1 == 65535);
	assert(c.t2 == -1 && c.t3 == -1 && c.p2 == -1 && c.p9 == -1);

	/* Byte order, and that every word lands in its own field. */
	for (int i = 0; i < BMP280_CALIB_LEN; i++)
		b[i] = (uint8_t)(i + 1);	/* 0x02 0x01, 0x04 0x03, ... */
	bmp280_parse_calib(b, &c);
	assert(c.t1 == 0x0201 && c.t2 == 0x0403 && c.t3 == 0x0605);
	assert(c.p1 == 0x0807 && c.p2 == 0x0A09 && c.p9 == 0x1817);
	assert(bmp280_calib_ok(&c));

	/* Either unsigned word reading zero is a trim image that never came. */
	c = CAL;
	c.t1 = 0;
	assert(!bmp280_calib_ok(&c));
	c = CAL;
	c.p1 = 0;
	assert(!bmp280_calib_ok(&c));
	c = CAL;
	c.t2 = 0;
	assert(bmp280_calib_ok(&c) && "a zero signed word is legal");
}

static void test_raw20(void)
{
	/* msb, lsb, then the top nibble of xlsb; the low nibble is ignored. */
	static const uint8_t b[6] = { 0x80, 0x00, 0x0F, 0x65, 0x5A, 0xC0 };

	assert(bmp280_raw20(&b[0]) == BMP280_RAW_RESET);
	assert(bmp280_raw20(&b[3]) == 0x655AC);
}

static void test_ctrl_meas(void)
{
	/* osrs_t[7:5] | osrs_p[4:2] | mode[1:0], the recommended pairs. */
	assert(bmp280_ctrl_meas(BMP280_OSR_ULTRA_HIGH, BMP280_MODE_FORCED) ==
	       0x55);
	assert(bmp280_ctrl_meas(BMP280_OSR_HIGH, BMP280_MODE_FORCED) == 0x31);
	assert(bmp280_ctrl_meas(BMP280_OSR_STANDARD, BMP280_MODE_FORCED) ==
	       0x2D);
	assert(bmp280_ctrl_meas(BMP280_OSR_LOW, BMP280_MODE_FORCED) == 0x29);
	assert(bmp280_ctrl_meas(BMP280_OSR_ULTRA_LOW, BMP280_MODE_FORCED) ==
	       0x25);
	assert(bmp280_ctrl_meas(BMP280_OSR_ULTRA_HIGH, BMP280_MODE_SLEEP) ==
	       0x54);
	assert(bmp280_ctrl_meas(5, BMP280_MODE_FORCED) == 0);
	assert(bmp280_ctrl_meas(-1, BMP280_MODE_FORCED) == 0);

	/*
	 * No preset may skip a field: osrs = 000 means the register is skipped
	 * and the result stays at 0x80000 forever.
	 */
	for (int osr = 0; osr <= 4; osr++) {
		uint8_t r = bmp280_ctrl_meas(osr, BMP280_MODE_FORCED);

		assert(((r >> 5) & 7) >= 1 && ((r >> 2) & 7) >= 1);
		assert((r & 3) == BMP280_MODE_FORCED);
	}
}

static void test_meas_ms(void)
{
	/*
	 * t_max = 1.25 + 2.3*osrs_t + (2.3*osrs_p + 0.575) ms, rounded up, with
	 * the oversampling multipliers decoded back out of ctrl_meas. Worked
	 * in microseconds so the table cannot be checked against itself.
	 */
	for (int osr = 0; osr <= 4; osr++) {
		uint8_t r = bmp280_ctrl_meas(osr, BMP280_MODE_FORCED);
		long ot = 1L << (((r >> 5) & 7) - 1);
		long op = 1L << (((r >> 2) & 7) - 1);
		long us = 1250 + 2300 * ot + 2300 * op + 575;

		assert(bmp280_meas_ms(osr) == (int)((us + 999) / 1000));
	}

	/* 43.2, 22.5, 13.3, 8.7, 6.4 ms rounded up. */
	assert(bmp280_meas_ms(BMP280_OSR_ULTRA_HIGH) == 44);
	assert(bmp280_meas_ms(BMP280_OSR_HIGH) == 23);
	assert(bmp280_meas_ms(BMP280_OSR_STANDARD) == 14);
	assert(bmp280_meas_ms(BMP280_OSR_LOW) == 9);
	assert(bmp280_meas_ms(BMP280_OSR_ULTRA_LOW) == 7);
	assert(bmp280_meas_ms(5) == 0 && bmp280_meas_ms(-1) == 0);
}

static void test_constants(void)
{
	assert(BMP280_CHIP_ID == 0x58 && BME280_CHIP_ID == 0x60);
	assert(BMP280_REG_ID == 0xD0 && BMP280_REG_RESET == 0xE0);
	assert(BMP280_RESET_WORD == 0xB6);
	assert(BMP280_REG_STATUS == 0xF3 && BMP280_REG_CTRL_MEAS == 0xF4);
	assert(BMP280_REG_CONFIG == 0xF5 && BMP280_REG_PRESS_MSB == 0xF7);
	assert(BMP280_REG_CALIB == 0x88 && BMP280_CALIB_LEN == 24);
	assert(BMP280_RESULT_LEN == 6 && BMP280_RAW_RESET == 0x80000);
	assert(BMP280_ADDR_SDO_LOW == 0x76 && BMP280_ADDR_SDO_HIGH == 0x77);
	assert(BMP280_STATUS_MEASURING == 0x08 &&
	       BMP280_STATUS_IM_UPDATE == 0x01);
}

int main(void)
{
	test_agrees_with_double();
	test_pinned_vectors();
	test_divide_by_zero_and_wraparound();
	test_corners();
	test_parse_calib();
	test_raw20();
	test_ctrl_meas();
	test_meas_ms();
	test_constants();
	printf("all bmp280 math checks passed\n");
	return 0;
}
