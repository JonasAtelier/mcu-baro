/*
 * Host-side check of the core: baro_read(), baro_altitude_m() and the log
 * macros, against a fake chip whose ops->read hands back whatever the test
 * put in it.
 *
 * The per-chip rules -- -EBUSY on a second probe, -EINVAL for another
 * driver's device -- live in each driver, so each test_<chip>.c checks them
 * against that driver's real probe.
 */
#include <assert.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"

#include "baro.h"
#include "baro_log.h"

static struct baro_sample fake_sample;
static int fake_ret;
static int fake_calls;

static int fake_read(struct baro_dev *baro, struct baro_sample *s)
{
	(void)baro;
	fake_calls++;
	/* The transfer takes time: a stamp taken after it would show this. */
	stub_time_us += 1000;
	if (fake_ret)
		return fake_ret;
	s->pressure_pa = fake_sample.pressure_pa;
	s->temp_c = fake_sample.temp_c;
	return 0;
}

static const struct baro_ops fake_ops = { .read = fake_read };

static struct baro_dev fake_dev(void)
{
	struct baro_dev baro = { .name = "fake", .ops = &fake_ops };

	fake_sample = (struct baro_sample){
		.pressure_pa = 100653.25f,
		.temp_c = 25.08f,
	};
	fake_ret = 0;
	fake_calls = 0;
	return baro;
}

static void test_read_stamps(void)
{
	struct baro_dev baro = fake_dev();
	struct baro_sample s = { .t_us = -1 };

	stub_time_us = 123456789;
	assert(baro_read(&baro, &s) == 0);
	assert(fake_calls == 1);
	/* Stamped before the read, not after: the fake moved the clock. */
	assert(s.t_us == 123456789);
	assert(stub_time_us == 123456789 + 1000);
	assert(s.pressure_pa == 100653.25f && s.temp_c == 25.08f);

	/* A new read, a new stamp. */
	stub_time_us = 123456999;
	assert(baro_read(&baro, &s) == 0);
	assert(s.t_us == 123456999);
}

static void test_errno_passthrough(void)
{
	struct baro_dev baro = fake_dev();
	struct baro_sample s = { .pressure_pa = 1.0f, .temp_c = 99.0f,
				 .t_us = 7 };

	/* Whatever the chip says comes back unchanged, and s is left alone. */
	stub_time_us = 42;
	fake_ret = -ETIMEDOUT;
	assert(baro_read(&baro, &s) == -ETIMEDOUT);
	fake_ret = -ENXIO;
	assert(baro_read(&baro, &s) == -ENXIO);
	fake_ret = -ERANGE;
	assert(baro_read(&baro, &s) == -ERANGE);
	fake_ret = -EIO;
	assert(baro_read(&baro, &s) == -EIO);
	assert(s.pressure_pa == 1.0f && s.temp_c == 99.0f && s.t_us == 7);

	/* Nothing to call is the caller's mistake, not a chip error. */
	fake_calls = 0;
	assert(baro_read(NULL, &s) == -EINVAL);
	assert(baro_read(&baro, NULL) == -EINVAL);
	baro.ops = NULL;
	assert(baro_read(&baro, &s) == -EINVAL);
	assert(fake_calls == 0);
	assert(s.t_us == 7);
}

/*
 * Against the ICAO standard atmosphere, which this formula approximates up
 * to the tropopause: 1000 m is 89874.6 Pa, 5000 m is 54019.9 Pa, 11000 m is
 * 22632.1 Pa. The formula's own constants round, so a couple of metres.
 */
static void test_altitude_known_values(void)
{
	assert(fabsf(baro_altitude_m(89874.6f, BARO_SEA_LEVEL_PA) - 1000.0f) <
	       2.0f);
	assert(fabsf(baro_altitude_m(54019.9f, BARO_SEA_LEVEL_PA) - 5000.0f) <
	       2.0f);
	assert(fabsf(baro_altitude_m(22632.1f, BARO_SEA_LEVEL_PA) - 11000.0f) <
	       2.0f);
	assert(BARO_SEA_LEVEL_PA == 101325.0f);
}

static void test_altitude(void)
{
	float h;

	/* At the reference pressure, altitude is zero. */
	assert(fabsf(baro_altitude_m(101325.0f, 101325.0f)) < 1e-3f);
	/* Lower pressure is higher up, roughly 8 m per hPa near sea level. */
	h = baro_altitude_m(101325.0f - 100.0f, 101325.0f);
	assert(h > 7.0f && h < 10.0f);
	assert(baro_altitude_m(50000.0f, 101325.0f) > 5000.0f);
	/* Higher pressure than the reference means below it. */
	assert(baro_altitude_m(102000.0f, 101325.0f) < 0.0f);
	/* Relative: the reference is wherever you took it. */
	assert(fabsf(baro_altitude_m(95000.0f, 95000.0f)) < 1e-3f);
	/* Nonsense in, zero out, never NaN. */
	assert(baro_altitude_m(0.0f, 101325.0f) == 0.0f);
	assert(baro_altitude_m(-1.0f, 101325.0f) == 0.0f);
	assert(baro_altitude_m(101325.0f, 0.0f) == 0.0f);
	assert(baro_altitude_m(NAN, 101325.0f) == 0.0f);
	assert(baro_altitude_m(101325.0f, NAN) == 0.0f);
}

static void test_log_prefix(void)
{
	struct baro_dev baro = fake_dev();
	struct baro_sample s;

	baro_err(&baro, "id at 0x%02x is 0x%02x", 0x76, 0x60);
	assert(!strcmp(stub_log_last, "E baro: fake: id at 0x76 is 0x60"));

	/* No arguments after the format must still compile and work. */
	baro_warn(&baro, "no arguments");
	assert(!strcmp(stub_log_last, "W baro: fake: no arguments"));
	baro_info(&baro, "i");
	assert(!strcmp(stub_log_last, "I baro: fake: i"));
	baro_dbg(&baro, "d");
	assert(!strcmp(stub_log_last, "D baro: fake: d"));

	stub_time_us = 42;
	assert(baro_read(&baro, &s) == 0);
	baro_log_sample(&baro, &s);
	assert(!strcmp(stub_log_last,
		       "I baro: fake: t 42 us   100653.2 Pa   25.08 C"));
}

int main(void)
{
	test_read_stamps();
	test_errno_passthrough();
	test_altitude_known_values();
	test_altitude();
	test_log_prefix();
	printf("all core checks passed\n");
	return 0;
}
