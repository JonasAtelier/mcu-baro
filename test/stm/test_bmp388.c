/*
 * Host-side check of the driver against a fake BMP388 behind a fake HAL. No
 * hardware, no ARM toolchain -- see test/Makefile.
 *
 * The fake is a 256-byte register file plus the two things on this chip that
 * take time: the start-up after a soft reset, during which the trim reads
 * zeros and EVENT.por_detected is not yet set, and a forced measurement,
 * during which the result registers hold whatever they held before and the
 * data-ready flags stay clear. It charges Table 22's *maximum* times, from
 * its own table in microseconds rather than the driver's rounded one, so the
 * driver cannot pass by agreeing with itself. EVENT and ERR_REG clear on
 * read; each drdy flag clears when one of its data registers is read.
 *
 * HAL_Delay(n) waits n + 1 ticks, the first of which may be partial, so all
 * it guarantees is n whole milliseconds. That is all the fake credits; the
 * tick counter HAL_GetTick() reads moves by all n + 1. The fake HAL also
 * insists on the HAL's 8-bit address form, the 7-bit address shifted left
 * one.
 */
#include <assert.h>
#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "stm32_hal_stub.h"

#include "baro.h"
#include "bmp388.h"
#include "bmp388_math.h"
#include "bmp388_regs.h"

/*
 * The same made-up trim set test_bmp388_math.c uses, as the chip stores it at
 * 0x31..0x45 (Table 23).
 */
static const uint8_t CAL_BYTES[BMP388_CALIB_LEN] = {
	0x6C, 0x6B,		/* T1  27500 */
	0x38, 0x4A,		/* T2  19000 */
	0xF9,			/* T3  -7 */
	0xF8, 0xF8,		/* P1  -1800 */
	0x8C, 0xF1,		/* P2  -3700 */
	0x24,			/* P3  36 */
	0x00,			/* P4  0 */
	0xD4, 0x62,		/* P5  25300 */
	0xBA, 0x77,		/* P6  30650 */
	0x03,			/* P7  3 */
	0xFA,			/* P8  -6 */
	0x80, 0x3E,		/* P9  16000 */
	0x0D,			/* P10 13 */
	0xC9,			/* P11 -55 */
};

/* Pinned, and self-generated, in test_bmp388_math.c. */
#define RAW_T		8450000
#define RAW_P		6500000
#define EXPECT_C	24.9007f
#define EXPECT_PA	97628.77f

#define RESET_US	2000	/* t_startup, Table 2 */

static I2C_HandleTypeDef bus;
static uint8_t regs[256];
static uint8_t chip_id;
static uint8_t cal[BMP388_CALIB_LEN];
static uint32_t raw_t, raw_p;	/* what the next finished measurement yields */
static long need_us, elapsed_us;
static enum { P_NONE, P_RESET, P_MEAS } pending;
static bool reset_hangs, meas_hangs;	/* the pending job never finishes */
static int cmd_busy_reads;	/* STATUS reads showing cmd_rdy clear; <0: all */
static uint8_t drdy_set;	/* the STATUS flags a finished measurement sets */
static uint16_t dev_addr;	/* 7-bit, of the last transfer */
static int writes, reads, resets, bursts;
static uint8_t last_pwr, last_osr, last_odr;
static HAL_StatusTypeDef bus_status;	/* every transfer returns this if set */
static HAL_StatusTypeDef probe_status;	/* IsDeviceReady on a present chip */
static bool absent;		/* nothing ACKs at the address */
static uint16_t probed_addr;
static int allocs, frees;

static void put24(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)v;
	p[1] = (uint8_t)(v >> 8);
	p[2] = (uint8_t)(v >> 16);
}

/*
 * A soft reset, as the fake models it: every user setting back to its
 * Table 24 default, results back to 0x800000, and the trim not there until
 * the start-up time is up. EVENT is left alone until then -- the datasheet
 * does not say when the reset touches it, and leaving a stale por_detected
 * standing is the case that would fool a driver.
 */
static void chip_soft_reset(void)
{
	memset(&regs[BMP388_REG_CALIB], 0, BMP388_CALIB_LEN);
	regs[BMP388_REG_ERR] = 0;
	regs[BMP388_REG_STATUS] = BMP388_STATUS_CMD_RDY;
	regs[BMP388_REG_PWR_CTRL] = 0;
	regs[BMP388_REG_OSR] = BMP388_OSR_RESET;
	regs[BMP388_REG_ODR] = 0;
	regs[BMP388_REG_CONFIG] = 0;
	put24(&regs[BMP388_REG_DATA], BMP388_RAW_RESET);
	put24(&regs[BMP388_REG_DATA + 3], BMP388_RAW_RESET);
	pending = P_RESET;
	need_us = RESET_US;
	elapsed_us = 0;
	resets++;
}

/* Apply whatever the chip was busy with, once enough time has gone by. */
static void settle(void)
{
	if (pending == P_NONE || elapsed_us < need_us)
		return;

	if (pending == P_RESET) {
		if (reset_hangs)
			return;
		memcpy(&regs[BMP388_REG_CALIB], cal, BMP388_CALIB_LEN);
		regs[BMP388_REG_EVENT] |= BMP388_EVENT_POR;
	} else {
		if (meas_hangs)
			return;
		put24(&regs[BMP388_REG_DATA], raw_p);
		put24(&regs[BMP388_REG_DATA + 3], raw_t);
		regs[BMP388_REG_STATUS] |= drdy_set;
		/* forced returns to sleep */
		regs[BMP388_REG_PWR_CTRL] &= (uint8_t)~0x30;
	}
	pending = P_NONE;
}

static void chip_power_on(void)
{
	absent = false;
	probed_addr = 0;
	memset(regs, 0, sizeof(regs));
	chip_id = BMP388_CHIP_ID;
	memcpy(cal, CAL_BYTES, sizeof(cal));
	raw_t = RAW_T;
	raw_p = RAW_P;
	reset_hangs = meas_hangs = false;
	cmd_busy_reads = 0;
	drdy_set = BMP388_STATUS_DRDY_PRESS | BMP388_STATUS_DRDY_TEMP;
	writes = reads = resets = bursts = 0;
	allocs = frees = 0;
	dev_addr = 0;
	last_pwr = last_osr = last_odr = 0xFF;
	bus_status = HAL_OK;
	probe_status = HAL_OK;
	/* Power-on is a reset that has already finished, por_detected set. */
	chip_soft_reset();
	elapsed_us = need_us;
	settle();
	resets = 0;
}

/* Table 22, maximum, in microseconds, by OSR register value. */
static long meas_max_us(uint8_t osr)
{
	switch (osr) {
	case 0x0D: return 78090;	/* p x32, t x2 */
	case 0x0C: return 41890;	/* p x16, t x2 */
	case 0x03: return 21530;	/* p x8,  t x1 */
	case 0x02: return 12480;	/* p x4,  t x1 */
	case 0x01: return 7960;		/* p x2,  t x1 */
	case 0x00: return 5700;		/* p x1,  t x1 */
	}
	assert(!"an OSR value Table 22 does not time");
	return 0;
}

/*
 * The address-only probe. The real one returns HAL_ERROR once every trial
 * went unACKed; a NACKed transfer is HAL_ERROR too, which the transfer fakes
 * below return for an absent chip.
 */
HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *hi2c,
					uint16_t address, uint32_t trials,
					uint32_t timeout)
{
	assert(hi2c == &bus);
	assert(!(address & 1) && "the HAL takes the 7-bit address << 1");
	assert(trials >= 1 && timeout > 0);
	probed_addr = address >> 1;
	if (absent)
		return HAL_ERROR;
	return probe_status;
}

HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c,
				    uint16_t address, uint16_t mem,
				    uint16_t mem_size, uint8_t *data,
				    uint16_t len, uint32_t timeout)
{
	uint8_t reg, val;

	assert(hi2c == &bus);
	assert(!(address & 1) && "the HAL takes the 7-bit address << 1");
	assert(mem_size == I2C_MEMADD_SIZE_8BIT && mem <= 0xFF);
	assert(len == 1 && "a write is the register address then one byte");
	assert(timeout > 0);
	if (absent)
		return HAL_ERROR;
	if (bus_status != HAL_OK)
		return bus_status;
	reg = (uint8_t)mem;
	val = data[0];
	writes++;
	dev_addr = address >> 1;

	if (reg == BMP388_REG_CMD) {
		if (val == BMP388_CMD_SOFTRESET)
			chip_soft_reset();
		return HAL_OK;
	}
	if (reg == BMP388_REG_OSR) {
		last_osr = val;
		regs[reg] = val;
		/*
		 * Pessimistic, as at the trigger below: an OSR slower than the
		 * ODR period latches conf_err the moment it is written, so
		 * writing OSR before ODR fails the first read.
		 */
		if (5000L << regs[BMP388_REG_ODR] < meas_max_us(val))
			regs[BMP388_REG_ERR] |= BMP388_ERR_CONF;
		return HAL_OK;
	}
	if (reg == BMP388_REG_ODR) {
		last_odr = val;
		regs[reg] = val;
		return HAL_OK;
	}
	assert(reg == BMP388_REG_PWR_CTRL && "the driver writes nothing else");
	last_pwr = val;
	regs[reg] = val;

	if ((val & 0x30) == 0x10 || (val & 0x30) == 0x20) {
		assert((val & 0x03) == 0x03 &&
		       "a disabled sensor never produces a result");
		need_us = meas_max_us(regs[BMP388_REG_OSR]);
		/*
		 * Pessimistic: an ODR period shorter than the measurement is
		 * flagged as conf_err even in forced mode. The datasheet does
		 * not say it is; this makes a driver that leaves ODR at its
		 * 5 ms reset value fail here rather than on some chip.
		 */
		if (5000L << regs[BMP388_REG_ODR] < need_us)
			regs[BMP388_REG_ERR] |= BMP388_ERR_CONF;
		elapsed_us = 0;
		pending = P_MEAS;
		regs[BMP388_REG_STATUS] &= (uint8_t)~(BMP388_STATUS_DRDY_PRESS |
						      BMP388_STATUS_DRDY_TEMP);
	}
	return HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t address,
				   uint16_t mem, uint16_t mem_size,
				   uint8_t *rx, uint16_t len,
				   uint32_t timeout)
{
	size_t rx_len = len;
	uint8_t first;

	assert(hi2c == &bus);
	assert(!(address & 1) && "the HAL takes the 7-bit address << 1");
	assert(mem_size == I2C_MEMADD_SIZE_8BIT && mem <= 0xFF);
	assert(timeout > 0);
	if (absent)
		return HAL_ERROR;
	if (bus_status != HAL_OK)
		return bus_status;
	reads++;
	dev_addr = address >> 1;
	settle();
	first = (uint8_t)mem;

	if (first == BMP388_REG_CHIP_ID) {
		assert(rx_len == 1);
		rx[0] = chip_id;
		return HAL_OK;
	}
	if (first == BMP388_REG_ERR) {
		assert(rx_len == BMP388_BURST_LEN &&
		       "flags and both results in one burst");
		bursts++;
	}
	assert((size_t)first + rx_len <= sizeof(regs));
	memcpy(rx, &regs[first], rx_len);	/* reads auto-increment */
	if (first == BMP388_REG_STATUS && rx_len == 1 && cmd_busy_reads) {
		rx[0] &= (uint8_t)~BMP388_STATUS_CMD_RDY;
		if (cmd_busy_reads > 0)
			cmd_busy_reads--;
	}

	/* The clear-on-read side effects, for whatever the read covered. */
	for (size_t i = 0; i < rx_len; i++) {
		size_t r = first + i;

		if (r == BMP388_REG_ERR)
			regs[r] &= (uint8_t)~(BMP388_ERR_CMD | BMP388_ERR_CONF);
		if (r == BMP388_REG_EVENT)
			regs[r] = 0;
		if (r >= 0x04 && r <= 0x06)
			regs[BMP388_REG_STATUS] &=
				(uint8_t)~BMP388_STATUS_DRDY_PRESS;
		if (r >= 0x07 && r <= 0x09)
			regs[BMP388_REG_STATUS] &=
				(uint8_t)~BMP388_STATUS_DRDY_TEMP;
	}
	return HAL_OK;
}

/*
 * HAL_Delay(n): the tick counter moves by n + 1, of which only n whole
 * milliseconds are sure, so that is all the chip is credited.
 */
void HAL_Delay(uint32_t ms)
{
	stub_tick += ms + 1;
	elapsed_us += (long)ms * 1000;
}

/* Counts the drivers' calloc() and free() calls; see test/Makefile. */
void *__real_calloc(size_t n, size_t size);
void *__wrap_calloc(size_t n, size_t size);
void *__wrap_calloc(size_t n, size_t size)
{
	allocs++;
	return __real_calloc(n, size);
}
void __real_free(void *p);
void __wrap_free(void *p);
void __wrap_free(void *p)
{
	frees++;
	__real_free(p);
}

static struct baro_dev open_ok(const struct bmp388_config *cfg)
{
	struct baro_dev baro = { 0 };

	chip_power_on();
	assert(bmp388_probe(&baro, &bus, cfg) == 0);
	assert(baro.ops && baro.priv);
	return baro;
}

/* ------------------------------------------------------------- tests ----- */

static void test_probe_defaults(void)
{
	struct baro_dev baro = open_ok(NULL);

	assert(!strcmp(baro.name, "bmp388"));
	assert(probed_addr == BMP388_ADDR_SDO_LOW && "NULL cfg selects 0x76");
	assert(dev_addr == BMP388_ADDR_SDO_LOW);
	assert(resets == 1 && "probe soft-resets once");
	assert(last_osr == 0x0D && "highest: p x32, t x2");
	assert(last_odr == BMP388_ODR_80MS && last_odr == 0x04 &&
	       "80 ms, past the slowest preset's 78.09 ms");
	assert(regs[BMP388_REG_ERR] == 0 &&
	       "ODR written before OSR: no conf_err latched");
	assert(last_pwr == 0xFF && regs[BMP388_REG_PWR_CTRL] == 0 &&
	       "the chip is left asleep");
	assert(regs[BMP388_REG_CONFIG] == 0 && "IIR filter bypassed");
	bmp388_remove(&baro);
	assert(allocs == 1 && frees == 1);
	assert(!baro.ops && !baro.priv && "remove zeroes the binding");
}

static void test_wrong_chip_id(void)
{
	struct baro_dev baro = { 0 };

	/*
	 * A BMP390 answers on the same addresses and is described as sharing
	 * the register map. Refused because this driver only claims the
	 * BMP388.
	 */
	chip_power_on();
	chip_id = BMP390_CHIP_ID;
	assert(bmp388_probe(&baro, &bus, NULL) == -ENODEV);
	assert(strstr(stub_log_last, "baro: bmp388: "));
	assert(strstr(stub_log_last, "that is a BMP390"));
	assert(resets == 0 && writes == 0 &&
	       "rejected before anything is written");
	assert(allocs == 1 && frees == 1 && "nothing leaked");
	assert(!baro.ops && !baro.priv && "a failed probe leaves it unbound");

	/* Anything else, including the 0x00 or 0xFF of a floating bus. */
	chip_power_on();
	chip_id = 0x00;
	assert(bmp388_probe(&baro, &bus, NULL) == -ENODEV);
	assert(!strstr(stub_log_last, "BMP390"));
	chip_power_on();
	chip_id = 0xFF;
	assert(bmp388_probe(&baro, &bus, NULL) == -ENODEV);
	chip_power_on();
	chip_id = 0x58;			/* a BMP280 at the same address */
	assert(bmp388_probe(&baro, &bus, NULL) == -ENODEV);
	assert(allocs == frees);
}

static void test_config(void)
{
	struct bmp388_config c = { .address = BMP388_ADDR_SDO_HIGH,
				   .osr = BMP388_OSR_ULTRA_LOW };
	struct baro_dev baro = open_ok(&c);
	struct baro_sample s;

	assert(probed_addr == BMP388_ADDR_SDO_HIGH);
	assert(dev_addr == BMP388_ADDR_SDO_HIGH);
	assert(last_osr == 0x00 && "ultra low is x1/x1, written anyway");
	assert(baro_read(&baro, &s) == 0);
	assert(last_pwr == 0x13 && "press_en, temp_en, forced");
	bmp388_remove(&baro);

	/* Refused before anything touches the bus, and nothing allocated. */
	chip_power_on();
	c.address = 0x60;
	assert(bmp388_probe(&baro, &bus, &c) == -EINVAL);
	c.address = 0;
	c.osr = (enum bmp388_osr)6;
	assert(bmp388_probe(&baro, &bus, &c) == -EINVAL);
	c.osr = (enum bmp388_osr)-1;
	assert(bmp388_probe(&baro, &bus, &c) == -EINVAL);
	assert(writes == 0 && reads == 0 && probed_addr == 0 && allocs == 0);
	assert(!baro.ops && !baro.priv);
}

static void test_read(void)
{
	struct baro_dev baro = open_ok(NULL);
	struct baro_sample s;

	writes = bursts = 0;
	stub_tick = 5555;
	assert(baro_read(&baro, &s) == 0);
	assert(writes == 1 && bursts == 1 && "one trigger, one burst");
	assert(last_pwr == BMP388_PWR_FORCED);
	assert(fabsf(s.temp_c - EXPECT_C) < 0.0005f);
	assert(fabsf(s.pressure_pa - EXPECT_PA) < 0.05f);
	assert(s.t_us == 5555000);
	assert(regs[BMP388_REG_PWR_CTRL] == 0x03 && "back asleep");

	/* A second read triggers again: forced mode measures once and sleeps. */
	raw_p = 7000000;
	raw_t = 7000000;
	assert(baro_read(&baro, &s) == 0);
	assert(writes == 2 && bursts == 2);
	assert(fabsf(s.temp_c - -0.7078f) < 0.0005f);
	assert(fabsf(s.pressure_pa - 83127.49f) < 0.05f);
	bmp388_remove(&baro);
}

static void test_waits_long_enough_at_every_osr(void)
{
	/*
	 * HAL_Delay() only takes whole milliseconds. Each wait must still cover
	 * Table 22's unrounded maximum, or the fake has not set drdy yet and the
	 * read fails.
	 */
	for (int osr = BMP388_OSR_HIGHEST; osr <= BMP388_OSR_ULTRA_LOW; osr++) {
		struct bmp388_config c = { .osr = (enum bmp388_osr)osr };
		struct baro_dev baro = open_ok(&c);
		struct baro_sample s;

		assert(baro_read(&baro, &s) == 0);
		assert(fabsf(s.pressure_pa - EXPECT_PA) < 0.05f);
		assert(pending == P_NONE && elapsed_us >= need_us &&
		       "waited it out");
		bmp388_remove(&baro);
	}
}

static void test_reset(void)
{
	struct baro_dev baro = { 0 };

	/*
	 * The trim reads zeros and por_detected stays clear until the start-up
	 * time is up. Probe must wait it out.
	 */
	baro = open_ok(NULL);
	assert(pending == P_NONE &&
	       "the reset had finished before the trim read");
	assert(regs[BMP388_REG_EVENT] == 0 && "and por_detected was read");
	bmp388_remove(&baro);

	/*
	 * A reset that never finishes. por_detected is still set from
	 * power-up, so a driver that did not clear it first would be fooled.
	 */
	chip_power_on();
	assert(regs[BMP388_REG_EVENT] == BMP388_EVENT_POR);
	reset_hangs = true;
	assert(bmp388_probe(&baro, &bus, NULL) == -EIO);
	assert(strstr(stub_log_last, "not reported complete"));
	assert(allocs == 1 && frees == 1 && !baro.ops && !baro.priv);
}

static void test_cmd_rdy(void)
{
	struct baro_dev baro = { 0 };
	uint32_t t0;

	/* Busy for a couple of polls, then ready: probe waits and goes on. */
	chip_power_on();
	cmd_busy_reads = 2;
	assert(bmp388_probe(&baro, &bus, NULL) == 0);
	assert(cmd_busy_reads == 0 && resets == 1);
	bmp388_remove(&baro);

	/*
	 * Never ready: -ETIMEDOUT, and no reset sent to a decoder that would
	 * not take it. The first HAL_GetTick() millisecond may be partial, so
	 * the 10 ms limit takes 11 counted to be sure of.
	 */
	chip_power_on();
	cmd_busy_reads = -1;
	t0 = stub_tick;
	assert(bmp388_probe(&baro, &bus, NULL) == -ETIMEDOUT);
	assert(stub_tick - t0 > 10 && "waited out the whole limit");
	assert(stub_tick - t0 <= 10 + 2 && "and not much more");
	assert(strstr(stub_log_last, "never reports cmd_rdy"));
	assert(resets == 0 && allocs == 1 && frees == 1);
	assert(!baro.ops && !baro.priv);
}

static void test_odr(void)
{
	struct baro_dev baro = open_ok(NULL);
	struct baro_sample s = { .pressure_pa = 1.0f };

	/*
	 * ODR back at its 5 ms reset value, as if probe had never set it: the
	 * fake flags conf_err on a 78 ms measurement, and that is -EIO, not a
	 * sample.
	 */
	regs[BMP388_REG_ODR] = 0x00;
	assert(baro_read(&baro, &s) == -EIO);
	assert(strstr(stub_log_last, "ERR_REG 0x04"));
	assert(s.pressure_pa == 1.0f && "sample untouched");

	regs[BMP388_REG_ODR] = BMP388_ODR_80MS;
	assert(baro_read(&baro, &s) == 0);
	bmp388_remove(&baro);
}

static void test_bad_trim(void)
{
	struct baro_dev baro = { 0 };

	/* A bus that reads nothing but zeros. */
	chip_power_on();
	memset(cal, 0, sizeof(cal));
	assert(bmp388_probe(&baro, &bus, NULL) == -EIO);
	assert(allocs == frees);
	assert(!baro.ops && !baro.priv);

	/* NVM_PAR_T1 = 0 alone. */
	chip_power_on();
	cal[0] = cal[1] = 0;
	assert(bmp388_probe(&baro, &bus, NULL) == -EIO);
	assert(allocs == frees);

	/* NVM_PAR_P5 = 0 alone. */
	chip_power_on();
	cal[11] = cal[12] = 0;
	assert(bmp388_probe(&baro, &bus, NULL) == -EIO);
	assert(allocs == frees);

	/* A signed coefficient reading zero is legal and must not be rejected. */
	chip_power_on();
	cal[4] = 0;			/* T3 */
	assert(bmp388_probe(&baro, &bus, NULL) == 0);
	bmp388_remove(&baro);
}

static void test_err_reg(void)
{
	struct baro_dev baro = open_ok(NULL);
	struct baro_sample s = { .pressure_pa = 1.0f, .temp_c = 1.0f,
				 .t_us = 3 };

	/* fatal_err sticks: every read fails. */
	regs[BMP388_REG_ERR] = BMP388_ERR_FATAL;
	assert(baro_read(&baro, &s) == -EIO);
	assert(strstr(stub_log_last, "ERR_REG 0x01"));
	assert(baro_read(&baro, &s) == -EIO);
	assert(s.pressure_pa == 1.0f && s.temp_c == 1.0f && s.t_us == 3 &&
	       "sample untouched");

	/* conf_err and cmd_err clear on read: one failed read, then fine. */
	regs[BMP388_REG_ERR] = BMP388_ERR_CONF;
	assert(baro_read(&baro, &s) == -EIO);
	assert(baro_read(&baro, &s) == 0);
	regs[BMP388_REG_ERR] = BMP388_ERR_CMD;
	assert(baro_read(&baro, &s) == -EIO);
	assert(baro_read(&baro, &s) == 0);
	bmp388_remove(&baro);
}

static void test_no_new_data(void)
{
	struct baro_dev baro = open_ok(NULL);
	struct baro_sample s = { .pressure_pa = 1.0f, .temp_c = 1.0f };

	/*
	 * A measurement that never finishes: the registers still hold the
	 * reset value and drdy is clear. Reported, not compensated.
	 */
	meas_hangs = true;
	assert(baro_read(&baro, &s) == -EIO);
	assert(strstr(stub_log_last, "no new measurement"));
	assert(s.pressure_pa == 1.0f && "sample untouched");
	meas_hangs = false;

	/* One flag alone is not enough either: half a sample is stale. */
	drdy_set = BMP388_STATUS_DRDY_PRESS;
	assert(baro_read(&baro, &s) == -EIO);
	drdy_set = BMP388_STATUS_DRDY_TEMP;
	assert(baro_read(&baro, &s) == -EIO);
	assert(s.pressure_pa == 1.0f && "sample untouched");

	drdy_set = BMP388_STATUS_DRDY_PRESS | BMP388_STATUS_DRDY_TEMP;
	assert(baro_read(&baro, &s) == 0);
	bmp388_remove(&baro);
}

static void test_pressure_out_of_range(void)
{
	struct baro_dev baro = open_ok(NULL);
	struct baro_sample s = { .pressure_pa = 1.0f, .temp_c = 1.0f };

	/*
	 * Raw pressure at the top of its 24-bit range compensates to about
	 * -83 kPa on this trim. Reported as an error, not as a number.
	 */
	raw_p = 0xFFFFFF;
	assert(baro_read(&baro, &s) == -ERANGE);
	assert(s.pressure_pa == 1.0f && "sample untouched");
	raw_p = RAW_P;
	assert(baro_read(&baro, &s) == 0);
	bmp388_remove(&baro);
}

static void test_bus_error(void)
{
	struct baro_dev baro = { 0 };
	struct baro_sample s;

	/*
	 * Nothing ACKs: -ENXIO, decided by HAL_I2C_IsDeviceReady() before
	 * anything is allocated or sent.
	 */
	chip_power_on();
	absent = true;
	assert(bmp388_probe(&baro, &bus, NULL) == -ENXIO);
	assert(probed_addr == BMP388_ADDR_SDO_LOW);
	assert(allocs == 0 && writes == 0 && reads == 0);
	assert(!baro.ops && !baro.priv);

	/* A bus that is busy or hangs during the presence check is not absence. */
	chip_power_on();
	probe_status = HAL_BUSY;	/* also: a handle never initialised */
	assert(bmp388_probe(&baro, &bus, NULL) == -EAGAIN);
	assert(allocs == 0 && writes == 0 && reads == 0);
	chip_power_on();
	probe_status = HAL_TIMEOUT;
	assert(bmp388_probe(&baro, &bus, NULL) == -ETIMEDOUT);
	assert(allocs == 0);

	/* A NACK in the middle of a transfer is an I/O error, not absence. */
	chip_power_on();
	bus_status = HAL_ERROR;
	assert(bmp388_probe(&baro, &bus, NULL) == -EIO);
	assert(allocs == 1 && frees == 1);
	chip_power_on();
	/* Not -EBUSY: from a probe that means "already bound". */
	bus_status = HAL_BUSY;
	assert(bmp388_probe(&baro, &bus, NULL) == -EAGAIN);
	assert(allocs == 1 && frees == 1);
	assert(!baro.ops && !baro.priv);

	/*
	 * A bus that dies after probe: the read reports it rather than the
	 * stale register contents.
	 */
	baro = open_ok(NULL);
	bus_status = HAL_TIMEOUT;
	assert(baro_read(&baro, &s) == -ETIMEDOUT);
	bus_status = HAL_OK;
	bmp388_remove(&baro);
}

static int other_read(struct baro_dev *baro, struct baro_sample *s)
{
	(void)baro;
	(void)s;
	return 0;
}

static void test_null_and_foreign_arguments(void)
{
	static const struct baro_ops other_ops = { .read = other_read };
	struct baro_dev other = { .name = "other", .ops = &other_ops,
				  .priv = regs };
	struct baro_dev baro = { 0 };

	chip_power_on();
	assert(bmp388_probe(NULL, &bus, NULL) == -EINVAL);
	assert(bmp388_probe(&baro, NULL, NULL) == -EINVAL);
	assert(allocs == 0 && writes == 0 && probed_addr == 0);
	bmp388_remove(NULL);
	bmp388_remove(&baro);			/* unprobed: a no-op */

	/* Another chip's baro_dev is not ours to remove. */
	bmp388_remove(&other);
	assert(other.ops == &other_ops && other.priv == regs);
	assert(frees == 0);
}

static void test_second_probe_is_refused(void)
{
	static const struct baro_ops other_ops = { .read = other_read };
	struct baro_dev other = { .name = "other", .ops = &other_ops };
	struct baro_dev baro = open_ok(NULL);
	void *priv = baro.priv;
	int w = writes, r = reads, n = allocs;
	struct baro_sample s;

	probed_addr = 0;
	assert(bmp388_probe(&baro, &bus, NULL) == -EBUSY);
	assert(probed_addr == 0 && writes == w && reads == r &&
	       "no bus traffic");
	assert(allocs == n && "no allocation");
	assert(baro.priv == priv && baro_read(&baro, &s) == 0 &&
	       "first binding intact");
	bmp388_remove(&baro);

	/* Bound by another driver: refused too, and left exactly as it was. */
	assert(bmp388_probe(&other, &bus, NULL) == -EBUSY);
	assert(!strcmp(other.name, "other") && other.ops == &other_ops);

	/* A removed device can be probed afresh. */
	chip_power_on();
	assert(bmp388_probe(&baro, &bus, NULL) == 0);
	bmp388_remove(&baro);
}

int main(void)
{
	test_probe_defaults();
	test_wrong_chip_id();
	test_config();
	test_read();
	test_waits_long_enough_at_every_osr();
	test_reset();
	test_cmd_rdy();
	test_odr();
	test_bad_trim();
	test_err_reg();
	test_no_new_data();
	test_pressure_out_of_range();
	test_bus_error();
	test_null_and_foreign_arguments();
	test_second_probe_is_refused();
	printf("all bmp388 driver checks passed\n");
	return 0;
}
