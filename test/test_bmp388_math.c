/*
 * Host-side check of the trim unpacking, the §9.1 scaling, the compensation
 * and the timing tables. Altitude is the core's and is checked in
 * test_core.c.
 *
 * The BMP388 datasheet publishes no worked example -- no trim set with raw
 * values and the answers next to them -- so there is nothing to reproduce
 * exactly, and nothing here claims to. What it does publish is the listing
 * in §9, in single precision. test_agrees_with_double() runs the driver's
 * single-precision copy against a double-precision transcription of the
 * same listing over a sweep, which bounds what float rounding costs.
 *
 * The fixed vectors in test_pinned_vectors() are SELF-GENERATED: this
 * implementation's own output, captured so that a later edit that changes a
 * number has to say so. They are not datasheet values and are not evidence
 * that the compensation is right, only that it has not moved.
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "bmp388.h"
#include "bmp388_math.h"
#include "bmp388_regs.h"

/*
 * A plausible trim set: made up for the test, tuned so that the raw values
 * below land near room temperature and sea-level pressure. There is no
 * published BMP388 vector to use instead.
 */
static const struct bmp388_calib CAL = {
	.t1 = 27500, .t2 = 19000, .t3 = -7,
	.p1 = -1800, .p2 = -3700, .p3 = 36, .p4 = 0,
	.p5 = 25300, .p6 = 30650, .p7 = 3, .p8 = -6,
	.p9 = 16000, .p10 = 13, .p11 = -55,
};

/* -------------------------- §9 again, in double precision ---- */

struct par_double {
	double t1, t2, t3;
	double p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11;
};

/* §9.1 with pow(), so it shares nothing with the hex-float literals. */
static void scale_double(const struct bmp388_calib *c, struct par_double *p)
{
	p->t1 = c->t1 / pow(2, -8);
	p->t2 = c->t2 / pow(2, 30);
	p->t3 = c->t3 / pow(2, 48);
	p->p1 = (c->p1 - pow(2, 14)) / pow(2, 20);
	p->p2 = (c->p2 - pow(2, 14)) / pow(2, 29);
	p->p3 = c->p3 / pow(2, 32);
	p->p4 = c->p4 / pow(2, 37);
	p->p5 = c->p5 / pow(2, -3);
	p->p6 = c->p6 / pow(2, 6);
	p->p7 = c->p7 / pow(2, 8);
	p->p8 = c->p8 / pow(2, 15);
	p->p9 = c->p9 / pow(2, 48);
	p->p10 = c->p10 / pow(2, 48);
	p->p11 = c->p11 / pow(2, 65);
}

static double temperature_double(const struct par_double *p, uint32_t u)
{
	double d1 = (double)u - p->t1;

	return d1 * p->t2 + d1 * d1 * p->t3;
}

static double pressure_double(const struct par_double *p, uint32_t up,
			      double t)
{
	double u = up;
	double out1 = p->p5 + p->p6 * t + p->p7 * t * t + p->p8 * t * t * t;
	double out2 = u * (p->p1 + p->p2 * t + p->p3 * t * t +
			   p->p4 * t * t * t);

	return out1 + out2 + u * u * (p->p9 + p->p10 * t) + u * u * u * p->p11;
}

/* -------------------------------------------------------------- tests ---- */

/*
 * Single against double, over raw values spanning about -30 to +80 C and
 * 30 to 160 kPa on this trim. Steps are prime-ish so the sweep does not
 * settle on round numbers.
 *
 * The bounds are what this sweep actually produces, rounded up: 0.000004 C
 * and 0.036 Pa over 17 338 pairs, 0.033 Pa built with fused multiply-add.
 * Float has a 24-bit mantissa and the pressure is a sum of terms near 2e5 Pa,
 * so a few hundredths of a pascal is the floor; the part's own resolution at
 * the highest oversampling is 0.085 Pa (Table 6).
 */
static void test_agrees_with_double(void)
{
	struct bmp388_par par;
	struct par_double pd;
	double worst_t = 0.0, worst_p = 0.0;
	int checked = 0;

	bmp388_scale_calib(&CAL, &par);
	scale_double(&CAL, &pd);

	for (uint32_t ut = 4500000; ut <= 11000000; ut += 35711) {
		float tf = bmp388_compensate_temperature(&par, ut);
		double td = temperature_double(&pd, ut);

		if (fabs(tf - td) > worst_t)
			worst_t = fabs(tf - td);

		for (uint32_t up = 2000000; up <= 12000000; up += 79193) {
			float pf = bmp388_compensate_pressure(&par, up, tf);
			double pdd = pressure_double(&pd, up, td);

			if (pdd < 30000.0 || pdd > 160000.0)
				continue;
			if (fabs(pf - pdd) > worst_p)
				worst_p = fabs(pf - pdd);
			checked++;
		}
	}

	assert(checked > 5000);
	assert(worst_t <= 0.00001 && "float and double temperature agree");
	assert(worst_p <= 0.05 && "float and double pressure agree");
}

/*
 * Fixed vectors, SELF-GENERATED from this implementation -- not datasheet
 * numbers. Each is cross-checked against the double transcription as well,
 * since pinning a wrong number is no use. Compared with a tolerance rather
 * than exactly: the compiler may fuse a multiply and add (the ESP32-S3 FPU
 * has madd.s, a Cortex-M4F or M7 vfma.f32), which moves the last bit.
 */
static void test_pinned_vectors(void)
{
	static const struct {
		uint32_t adc_t, adc_p;
		float temp_c, press_pa;
	} v[] = {
		{  8450000, 6500000,  24.9007f,  97628.77f },
		{  7000000, 7000000,  -0.7078f,  83127.49f },
		{  9500000, 5500000,  43.3795f, 120496.38f },
		{  6000000, 9000000, -18.4298f,  47183.69f },
		{ 10500000, 8000000,  60.9274f,  77917.39f },
	};
	struct bmp388_par par;
	struct par_double pd;

	bmp388_scale_calib(&CAL, &par);
	scale_double(&CAL, &pd);
	for (unsigned i = 0; i < sizeof(v) / sizeof(v[0]); i++) {
		float t = bmp388_compensate_temperature(&par, v[i].adc_t);
		float p = bmp388_compensate_pressure(&par, v[i].adc_p, t);
		double td = temperature_double(&pd, v[i].adc_t);

		assert(fabsf(t - v[i].temp_c) < 0.0005f);
		assert(fabsf(p - v[i].press_pa) < 0.05f);
		assert(fabs(pressure_double(&pd, v[i].adc_p, td) -
			    v[i].press_pa) < 0.1);
	}
}

static void test_scaling(void)
{
	/*
	 * §9.1 is exact: one NVM count is exactly 2^-n, and each offset
	 * subtracted before scaling is exactly 2^14.
	 */
	struct bmp388_calib c = { 0 };
	struct bmp388_par p;

	c.t1 = 1; c.t2 = 1; c.t3 = 1;
	c.p1 = 16384; c.p2 = 16384; c.p3 = 1; c.p4 = 1; c.p5 = 1; c.p6 = 1;
	c.p7 = 1; c.p8 = 1; c.p9 = 1; c.p10 = 1; c.p11 = 1;
	bmp388_scale_calib(&c, &p);
	assert(p.t1 == 256.0f && p.t2 == ldexpf(1, -30) &&
	       p.t3 == ldexpf(1, -48));
	assert(p.p1 == 0.0f && p.p2 == 0.0f && "NVM_PAR_Px - 2^14");
	assert(p.p3 == ldexpf(1, -32) && p.p4 == ldexpf(1, -37));
	assert(p.p5 == 8.0f && p.p6 == ldexpf(1, -6));
	assert(p.p7 == ldexpf(1, -8) && p.p8 == ldexpf(1, -15));
	assert(p.p9 == ldexpf(1, -48) && p.p10 == ldexpf(1, -48));
	assert(p.p11 == ldexpf(1, -65) && p.p11 > 0.0f && "not flushed to 0");

	/* The extremes, still exact: 65535 * 256 fits a float's mantissa. */
	c.t1 = 65535;
	c.p1 = -32768;
	c.p11 = -128;
	bmp388_scale_calib(&c, &p);
	assert(p.t1 == 16776960.0f);
	assert(p.p1 == -49152.0f / 1048576.0f);
	assert(p.p11 == -ldexpf(1, -58));
}

static void test_parse_calib(void)
{
	/*
	 * T1, T2, P5, P6 unsigned; T3, P3, P4, P7, P8, P10, P11 signed bytes;
	 * P1, P2, P9 signed words (Table 23). All 0xFF shows which is which.
	 */
	uint8_t b[BMP388_CALIB_LEN];
	struct bmp388_calib c;

	memset(b, 0xFF, sizeof(b));
	bmp388_parse_calib(b, &c);
	assert(c.t1 == 65535 && c.t2 == 65535 && c.p5 == 65535 &&
	       c.p6 == 65535);
	assert(c.t3 == -1 && c.p3 == -1 && c.p4 == -1 && c.p7 == -1 &&
	       c.p8 == -1 && c.p10 == -1 && c.p11 == -1);
	assert(c.p1 == -1 && c.p2 == -1 && c.p9 == -1);

	/* Byte order, and that every coefficient lands at its address. */
	for (int i = 0; i < BMP388_CALIB_LEN; i++)
		b[i] = (uint8_t)(BMP388_REG_CALIB + i);	/* 0x31..0x45 */
	bmp388_parse_calib(b, &c);
	assert(c.t1 == 0x3231 && c.t2 == 0x3433 && c.t3 == 0x35);
	assert(c.p1 == 0x3736 && c.p2 == 0x3938 && c.p3 == 0x3A &&
	       c.p4 == 0x3B);
	assert(c.p5 == 0x3D3C && c.p6 == 0x3F3E && c.p7 == 0x40 &&
	       c.p8 == 0x41);
	assert(c.p9 == 0x4342 && c.p10 == 0x44 && c.p11 == 0x45);
	assert(bmp388_calib_ok(&c));

	/* Either offset reading zero is a trim image that never came. */
	c = CAL;
	c.t1 = 0;
	assert(!bmp388_calib_ok(&c));
	c = CAL;
	c.p5 = 0;
	assert(!bmp388_calib_ok(&c));
	c = CAL;
	c.t2 = 0;
	c.p1 = 0;
	assert(bmp388_calib_ok(&c) && "other zero words are not checked");
}

static void test_raw24(void)
{
	/* xlsb, lsb, msb: little-endian, unsigned (Tables 28, 29). */
	static const uint8_t b[6] = { 0x00, 0x00, 0x80, 0xA0, 0xEF, 0x80 };

	assert(bmp388_raw24(&b[0]) == BMP388_RAW_RESET);
	assert(bmp388_raw24(&b[3]) == 0x80EFA0);
}

static void test_osr_reg(void)
{
	/* osr_t[5:3] | osr_p[2:0], Table 6's recommended pairs. */
	assert(bmp388_osr_reg(BMP388_OSR_HIGHEST) == 0x0D);
	assert(bmp388_osr_reg(BMP388_OSR_ULTRA_HIGH) == 0x0C);
	assert(bmp388_osr_reg(BMP388_OSR_HIGH) == 0x03);
	assert(bmp388_osr_reg(BMP388_OSR_STANDARD) == 0x02);
	assert(bmp388_osr_reg(BMP388_OSR_LOW) == 0x01);
	assert(bmp388_osr_reg(BMP388_OSR_ULTRA_LOW) == 0x00);
	assert(bmp388_osr_reg(BMP388_OSR_STANDARD) == BMP388_OSR_RESET &&
	       "the reset value is the standard preset");

	/* No field above x32 (101), the highest Table 41 defines. */
	for (int osr = 0; osr <= 5; osr++) {
		uint8_t r = bmp388_osr_reg(osr);

		assert((r & 7) <= 5 && ((r >> 3) & 7) <= 5 && (r & 0xC0) == 0);
	}
}

static void test_meas_ms(void)
{
	/*
	 * Table 22, maximum measurement time, in microseconds, so the driver's
	 * rounded table cannot be checked against itself.
	 */
	static const long max_us[6] = { 78090, 41890, 21530, 12480, 7960, 5700 };

	for (int osr = 0; osr <= 5; osr++)
		assert(bmp388_meas_ms(osr) == (int)((max_us[osr] + 999) / 1000));
	assert(bmp388_meas_ms(BMP388_OSR_HIGHEST) == 79);
	assert(bmp388_meas_ms(BMP388_OSR_ULTRA_LOW) == 6);
	assert(bmp388_meas_ms(6) == 0 && bmp388_meas_ms(-1) == 0);
}

static void test_constants(void)
{
	assert(BMP388_CHIP_ID == 0x50 && BMP390_CHIP_ID == 0x60);
	assert(BMP388_REG_CHIP_ID == 0x00 && BMP388_REG_ERR == 0x02);
	assert(BMP388_REG_STATUS == 0x03 && BMP388_REG_DATA == 0x04);
	assert(BMP388_REG_EVENT == 0x10 && BMP388_REG_PWR_CTRL == 0x1B);
	assert(BMP388_REG_OSR == 0x1C && BMP388_REG_ODR == 0x1D);
	assert(BMP388_REG_CONFIG == 0x1F && BMP388_REG_CMD == 0x7E);
	assert(BMP388_REG_CALIB == 0x31 && BMP388_CALIB_LEN == 21);
	assert(BMP388_REG_CALIB + BMP388_CALIB_LEN - 1 == 0x45);
	assert(BMP388_REG_ERR + BMP388_BURST_LEN - 1 == 0x09 &&
	       "the burst ends on the last temperature byte");
	assert(BMP388_CMD_SOFTRESET == 0xB6);
	assert(BMP388_PWR_FORCED == 0x13 && "press_en, temp_en, mode 01");
	assert(BMP388_ADDR_SDO_LOW == 0x76 && BMP388_ADDR_SDO_HIGH == 0x77);
	assert(BMP388_STATUS_CMD_RDY == 0x10 &&
	       BMP388_STATUS_DRDY_PRESS == 0x20 &&
	       BMP388_STATUS_DRDY_TEMP == 0x40);
	assert(BMP388_ERR_FATAL == 0x01 && BMP388_ERR_CMD == 0x02 &&
	       BMP388_ERR_CONF == 0x04);
	assert(BMP388_EVENT_POR == 0x01 && BMP388_RAW_RESET == 0x800000);
}

int main(void)
{
	test_agrees_with_double();
	test_pinned_vectors();
	test_scaling();
	test_parse_calib();
	test_raw24();
	test_osr_reg();
	test_meas_ms();
	test_constants();
	printf("all bmp388 math checks passed\n");
	return 0;
}
