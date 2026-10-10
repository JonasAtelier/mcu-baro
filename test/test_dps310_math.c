/*
 * Host-side check of the coefficient unpacking and compensation. Altitude is
 * the core's and is checked in test_core.c.
 *
 * The coefficient widths are 12, 12, 20, 20 and then 16 bits, packed into
 * eighteen bytes with three of them straddling a nibble boundary. That is
 * where a hand-written unpack goes wrong, and the symptom is a pressure that
 * is stable, plausible and tens of hectopascal out. So the unpacking is
 * tested against hand-computed bit patterns, not against itself.
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "dps310_math.h"
#include "dps310_regs.h"

#define NEAR(a, b) (fabs((double)(a) - (double)(b)) < 1e-3 * fabs(b) + 1e-6)

static void test_sign_extension(void)
{
	assert(dps310_sext(0xFFF, 12) == -1);
	assert(dps310_sext(0x7FF, 12) == 2047);
	assert(dps310_sext(0x800, 12) == -2048);
	assert(dps310_sext(0xFFFFF, 20) == -1);
	assert(dps310_sext(0x7FFFF, 20) == 524287);
	assert(dps310_sext(0x80000, 20) == -524288);
	assert(dps310_sext(0xFFFF, 16) == -1);
	assert(dps310_sext(0x8000, 16) == -32768);
	assert(dps310_sext(0xFFFFFF, 24) == -1);
	assert(dps310_sext(0x800000, 24) == -8388608);
	assert(dps310_sext(0x7FFFFF, 24) == 8388607);
	assert(dps310_sext(0, 12) == 0);
}

static void test_coefficient_unpacking(void)
{
	struct dps310_coefs k;
	uint8_t c[DPS310_COEF_LEN];

	/* All zeroes must give all zeroes -- no accidental sign bit. */
	memset(c, 0x00, sizeof(c));
	dps310_unpack_coefs(c, &k);
	assert(k.c0 == 0 && k.c1 == 0 && k.c00 == 0 && k.c10 == 0);
	assert(k.c01 == 0 && k.c11 == 0 && k.c20 == 0 && k.c21 == 0 &&
	       k.c30 == 0);

	/* All ones: every field is -1, which only holds if each width is right. */
	memset(c, 0xFF, sizeof(c));
	dps310_unpack_coefs(c, &k);
	assert(k.c0 == -1 && k.c1 == -1 && k.c00 == -1 && k.c10 == -1);
	assert(k.c01 == -1 && k.c11 == -1 && k.c20 == -1 && k.c21 == -1 &&
	       k.c30 == -1);

	/*
	 * A pattern with a distinct value in every field, worked out by hand:
	 *   c0  = 0x123 (291)       from c[0]=0x12, c[1] high nibble = 3
	 *   c1  = 0x456 (1110)      from c[1] low nibble = 4, c[2]=0x56
	 *   c00 = 0x789AB (494507)  from c[3]=0x78, c[4]=0x9A, c[5] high = B
	 *   c10 = 0xCDEF0 (-205072) from c[5] low = C, c[6]=0xDE, c[7]=0xF0
	 */
	memset(c, 0, sizeof(c));
	c[0] = 0x12; c[1] = 0x34; c[2] = 0x56;
	c[3] = 0x78; c[4] = 0x9A; c[5] = 0xBC; c[6] = 0xDE; c[7] = 0xF0;
	c[8] = 0x11; c[9] = 0x22;	/* c01 = 0x1122 */
	c[10] = 0x80; c[11] = 0x00;	/* c11 = most negative 16-bit */
	c[12] = 0x7F; c[13] = 0xFF;	/* c20 = most positive */
	c[14] = 0xFF; c[15] = 0xFF;	/* c21 = -1 */
	c[16] = 0x00; c[17] = 0x01;	/* c30 = 1 */
	dps310_unpack_coefs(c, &k);

	assert(k.c0 == 0x123);
	assert(k.c1 == 0x456);
	assert(k.c00 == 0x789AB);
	assert(k.c10 == dps310_sext(0xCDEF0, 20));
	assert(k.c10 < 0 && "0xC.. has the 20-bit sign bit set");
	assert(k.c01 == 0x1122);
	assert(k.c11 == -32768);
	assert(k.c20 == 32767);
	assert(k.c21 == -1);
	assert(k.c30 == 1);

	/* c0 and c1 share byte 1: changing one must not disturb the other. */
	memset(c, 0, sizeof(c));
	c[1] = 0xF0;
	dps310_unpack_coefs(c, &k);
	assert(k.c0 == 0x00F && k.c1 == 0);
	memset(c, 0, sizeof(c));
	c[1] = 0x0F;
	dps310_unpack_coefs(c, &k);
	/* 0xF00 has the 12-bit sign bit set, so this is negative. */
	assert(k.c0 == 0 && k.c1 == dps310_sext(0xF00, 12) && k.c1 == -256);

	/* c00 and c10 likewise share byte 5. */
	memset(c, 0, sizeof(c));
	c[5] = 0xF0;
	dps310_unpack_coefs(c, &k);
	assert(k.c00 == 0x0000F && k.c10 == 0);
	memset(c, 0, sizeof(c));
	c[5] = 0x0F;
	dps310_unpack_coefs(c, &k);
	/* likewise 0xF0000 against a 20-bit field */
	assert(k.c00 == 0 && k.c10 == dps310_sext(0xF0000, 20) &&
	       k.c10 == -65536);
}

static void test_scale_factors(void)
{
	/* The datasheet table, carried identically by both reference drivers. */
	assert(NEAR(dps310_scale_factor(0), 524288.0f));
	assert(NEAR(dps310_scale_factor(1), 1572864.0f));
	assert(NEAR(dps310_scale_factor(2), 3670016.0f));
	assert(NEAR(dps310_scale_factor(3), 7864320.0f));
	assert(NEAR(dps310_scale_factor(4), 253952.0f));
	assert(NEAR(dps310_scale_factor(5), 516096.0f));
	assert(NEAR(dps310_scale_factor(6), 1040384.0f));
	assert(NEAR(dps310_scale_factor(7), 2088960.0f));
	/*
	 * It is NOT monotonic -- 16x is smaller than 8x, because the result is
	 * shifted from there on. Guard that so nobody "fixes" the table.
	 */
	assert(dps310_scale_factor(4) < dps310_scale_factor(3));
	assert(dps310_scale_factor(8) == 0.0f);
	assert(dps310_scale_factor(-1) == 0.0f);

	/* The shift bit is needed exactly where the table drops. */
	assert(!dps310_needs_shift(3) && dps310_needs_shift(4));
	for (int p = 0; p <= 3; p++)
		assert(!dps310_needs_shift(p));
	for (int p = 4; p <= 7; p++)
		assert(dps310_needs_shift(p));

	assert(dps310_oversampling(0) == 1 && dps310_oversampling(7) == 128);
	assert(dps310_rate_hz(0) == 1 && dps310_rate_hz(7) == 128);
	assert(dps310_oversampling(8) == 0 && dps310_rate_hz(8) == 0);
}

static void test_busy_time(void)
{
	/* 3.6 ms ... 206.8 ms per measurement, in tenths of a millisecond. */
	static const int t[8] = { 36, 52, 84, 148, 276, 532, 1044, 2068 };

	for (int p = 0; p <= 7; p++)
		assert(dps310_busy_time(0, p) == t[p]);

	/* 64x at 16 Hz is 1.67 s of work per second: refused. */
	assert(dps310_busy_time(4, 6) == 16704 && 16704 >= DPS310_BUSY_LIMIT);
	/* The README and example config, 64x and 8x at 8 Hz: 953.6 ms, fits. */
	assert(dps310_busy_time(3, 6) + dps310_busy_time(3, 3) == 9536);
	assert(9536 < DPS310_BUSY_LIMIT);
}

static void test_raw24(void)
{
	const uint8_t pos[3] = { 0x00, 0x00, 0x01 };
	const uint8_t neg[3] = { 0xFF, 0xFF, 0xFF };
	const uint8_t big[3] = { 0x7F, 0xFF, 0xFF };
	const uint8_t low[3] = { 0x80, 0x00, 0x00 };

	/* Most significant byte first. */
	assert(dps310_raw24(pos) == 1);
	assert(dps310_raw24(neg) == -1);
	assert(dps310_raw24(big) == 8388607);
	assert(dps310_raw24(low) == -8388608);
}

static void test_compensation(void)
{
	/* A contrived but exactly checkable set. */
	struct dps310_coefs k = { .c0 = 100, .c1 = 200, .c00 = 90000,
				  .c10 = 1000, .c01 = 10, .c11 = 20, .c20 = 30,
				  .c21 = 40, .c30 = 50 };
	float p;

	/* T = c0*0.5 + c1*Traw_sc */
	assert(NEAR(dps310_temperature_c(&k, 0.0f), 50.0f));
	assert(NEAR(dps310_temperature_c(&k, 0.1f), 50.0f + 20.0f));

	/* With both scaled raws at zero, pressure is just c00. */
	assert(NEAR(dps310_pressure_pa(&k, 0.0f, 0.0f), 90000.0f));

	/*
	 * Hand-expanded at praw_sc = 0.5, traw_sc = 0.25:
	 * 90000 + 0.5*(1000 + 0.5*(30 + 0.5*50)) + 0.25*10
	 *       + 0.25*0.5*(20 + 0.5*40)
	 */
	p = 90000.0f + 0.5f * (1000.0f + 0.5f * (30.0f + 0.5f * 50.0f)) +
	    0.25f * 10.0f + 0.25f * 0.5f * (20.0f + 0.5f * 40.0f);
	assert(NEAR(dps310_pressure_pa(&k, 0.5f, 0.25f), p));

	/*
	 * Temperature genuinely moves the pressure -- this is why a wrong
	 * COEF_SRCE corrupts pressure as well as temperature.
	 */
	assert(dps310_pressure_pa(&k, 0.5f, 0.0f) !=
	       dps310_pressure_pa(&k, 0.5f, 0.5f));

	assert(dps310_traw_sc(1000, 0.0f) == 0.0f);	/* no divide by zero */
	assert(NEAR(dps310_traw_sc(524288, 524288.0f), 1.0f));
}

static void test_register_constants(void)
{
	assert(DPS310_PRODUCT_ID == 0x0D && DPS310_PRODUCT_ID_VALUE == 0x10);
	assert(DPS310_PRS_B2 == 0x00 && DPS310_TMP_B2 == 0x03);
	assert(DPS310_COEF_BASE == 0x10 && DPS310_COEF_LEN == 18);
	assert(DPS310_COEF_SRCE == 0x28 && DPS310_COEF_SRCE_MASK == 0x80);
	assert(DPS310_RESET == 0x0C && DPS310_RESET_MAGIC == 0x09);
	assert(DPS310_SENSOR_RDY == 0x40 && DPS310_COEF_RDY == 0x80);
	assert(DPS310_ADDR_LOW == 0x76 && DPS310_ADDR_HIGH == 0x77);
	assert(DPS310_ERRATA_PROBE == 0x32 && DPS310_ERRATA_OK_BIT == 0x02);
	/* Against the datasheet, not the Linux driver: P_SHIFT is bit 2. */
	assert(DPS310_PRS_SHIFT_EN == 0x04 && DPS310_TMP_SHIFT_EN == 0x08);
}

int main(void)
{
	test_sign_extension();
	test_coefficient_unpacking();
	test_scale_factors();
	test_busy_time();
	test_raw24();
	test_compensation();
	test_register_constants();
	printf("all dps310 math checks passed\n");
	return 0;
}
