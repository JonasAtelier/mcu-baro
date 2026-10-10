/*
 * Host-side check of the CRC and the compensation. Altitude is the core's and
 * is checked in test_core.c.
 *
 * The compensation is checked against the datasheet's own worked example,
 * then against two hand-worked cases for the second-order branches -- below
 * 20 C and below -15 C. Those only run in the cold, so a wrong constant there
 * stays hidden all summer.
 */
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "ms5611.h"
#include "ms5611_math.h"
#include "ms5611_proto.h"

/*
 * The datasheet's typical calibration, C1..C6 in words 1..6 as in PROM.
 * Words 0 and 7 do not enter the compensation.
 */
static const uint16_t DS[8] = { 0, 40127, 36924, 23317, 23282, 33464, 28312,
				0 };

static void test_datasheet_example(void)
{
	int32_t t, p;

	/* dT = 2366, OFF = 2420281617, SENS = 1315097036 on the way. */
	ms5611_compensate(DS, 9085466, 8569150, &t, &p);
	assert(t == 2007);		/* 20.07 C */
	assert(p == 100009);		/* 1000.09 mbar */
}

static void test_second_order_below_20c(void)
{
	int32_t t, p;

	/*
	 * D2 = 8304640, so dT = -262144 (-2^18).
	 * TEMP  = 2000 + dT*C6/2^23 = 2000 - 884 (884.75, truncated) = 1116
	 * T2    = dT^2/2^31 = 32                            -> TEMP = 1084
	 * OFF   = C2*2^16 + C4*dT/2^7 = 2419851264 - 47681536 = 2372169728
	 * SENS  = C1*2^15 + C3*dT/2^8 = 1314881536 - 23876608 = 1291004928
	 * OFF2  = 5*(TEMP-2000)^2/2 = 5*884^2/2 = 1953640   -> OFF  = 2370216088
	 * SENS2 = 5*(TEMP-2000)^2/4 = 976820                -> SENS = 1290028108
	 * P     = (9085466*SENS/2^21 - OFF)/2^15
	 *       = (5588773018 - 2370216088)/2^15 = 98222
	 */
	ms5611_compensate(DS, 9085466, 8304640, &t, &p);
	assert(t == 1084);
	assert(p == 98222);
}

static void test_second_order_below_minus_15c(void)
{
	int32_t t, p;

	/*
	 * Well below -15 C, so the extra terms are big enough to move P: just
	 * under the threshold (TEMP+1500)^2 is tiny and a wrong constant there
	 * changes P by less than 1 Pa, which the truncation hides.
	 *
	 * D2 = 6993920, so dT = -1572864 (-3*2^19).
	 * TEMP  = 2000 - 5308 (5308.5, truncated) = -3308
	 * T2    = dT^2/2^31 = 1152                          -> TEMP = -4460
	 * OFF   = 2419851264 - 286089216 = 2133762048
	 * SENS  = 1314881536 - 143259648 = 1171621888
	 * OFF2  = 5*5308^2/2 + 7*1808^2    = 70437160 + 22882048 = 93319208
	 * SENS2 = 5*5308^2/4 + 11*1808^2/2 = 35218580 + 17978752 = 53197332
	 * OFF   = 2040442840, SENS = 1118424556
	 * P     = (4845337046 - 2040442840)/2^15 = 85598
	 */
	ms5611_compensate(DS, 9085466, 6993920, &t, &p);
	assert(t == -4460);
	assert(p == 85598);
}

static void test_crc4(void)
{
	/* The example PROM and its CRC, as published in AN520 rev 004. */
	const uint16_t an520[8] = { 0x3132, 0x3334, 0x3536, 0x3738,
				    0x3940, 0x4142, 0x4344, 0x4500 };
	uint16_t p[8];

	assert(ms5611_crc4(an520) == 0x0B);

	/* The low byte of word 7 is taken as zero: that is where the CRC lives. */
	memcpy(p, an520, sizeof(p));
	p[7] = 0x4546;
	assert(ms5611_crc4(p) == 0x0B);

	/* A matching CRC passes; a wrong one does not. */
	memcpy(p, DS, sizeof(p));
	p[7] = ms5611_crc4(p);
	assert(ms5611_prom_ok(p));
	p[7] ^= 0x0001;
	assert(!ms5611_prom_ok(p));

	/* An all-zero PROM has CRC 0, so the CRC alone would accept a dead bus. */
	memset(p, 0, sizeof(p));
	assert(ms5611_crc4(p) == 0);
	assert(!ms5611_prom_ok(p));
}

static void test_osr_tables(void)
{
	/* COMMANDS: D1 0x40..0x48 and D2 0x50..0x58, lowest OSR first. */
	assert(MS5611_CMD_CONV_D1 + ms5611_osr_cmd(MS5611_OSR_4096) == 0x48);
	assert(MS5611_CMD_CONV_D2 + ms5611_osr_cmd(MS5611_OSR_4096) == 0x58);
	assert(MS5611_CMD_CONV_D1 + ms5611_osr_cmd(MS5611_OSR_1024) == 0x44);
	assert(MS5611_CMD_CONV_D1 + ms5611_osr_cmd(MS5611_OSR_256) == 0x40);
	assert(MS5611_CMD_CONV_D2 + ms5611_osr_cmd(MS5611_OSR_256) == 0x50);

	/* The datasheet maxima rounded up: 9.04, 4.54, 2.28, 1.17, 0.60 ms. */
	assert(ms5611_conv_ms(MS5611_OSR_4096) == 10);
	assert(ms5611_conv_ms(MS5611_OSR_2048) == 5);
	assert(ms5611_conv_ms(MS5611_OSR_1024) == 3);
	assert(ms5611_conv_ms(MS5611_OSR_512) == 2);
	assert(ms5611_conv_ms(MS5611_OSR_256) == 1);
	assert(ms5611_conv_ms(5) == 0 && ms5611_conv_ms(-1) == 0);
}

static void test_constants(void)
{
	assert(MS5611_CMD_RESET == 0x1E && MS5611_CMD_ADC_READ == 0x00);
	assert(MS5611_CMD_PROM_READ == 0xA0 && MS5611_PROM_WORDS == 8);
	assert(MS5611_ADDR_CSB_HIGH == 0x76 && MS5611_ADDR_CSB_LOW == 0x77);
}

int main(void)
{
	test_datasheet_example();
	test_second_order_below_20c();
	test_second_order_below_minus_15c();
	test_crc4();
	test_osr_tables();
	test_constants();
	printf("all ms5611 math checks passed\n");
	return 0;
}
