/*
 * Host-side check of the driver against a fake DPS310 behind a fake HAL. No
 * hardware, no ARM toolchain -- see test/Makefile.
 *
 * The fake models the two sequencing rules that matter: the coefficients are
 * meaningless until COEF_RDY is set, and the temperature source has to match
 * where the calibration came from. Both produce readings that look fine.
 *
 * The clock is simulated: only HAL_Delay() moves HAL_GetTick(), by n + 1
 * ticks for HAL_Delay(n), as the real one waits. test_ready_wait_timing()
 * pins the ready wait to the clock, across the tick counter's wrap.
 */
#include <assert.h>
#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "stm32_hal_stub.h"

#include "baro.h"
#include "dps310.h"
#include "dps310_math.h"
#include "dps310_regs.h"

static I2C_HandleTypeDef bus;
static uint8_t regs[256];
static uint16_t dev_addr;	/* 7-bit, of the last transfer */
static int reset_count, errata_writes, ready_delay;
/* MEAS_CFG polls that report SENSOR_RDY alone, coefficients reading 0 */
static int coef_delay;
static int allocs, frees, writes, reads;
static HAL_StatusTypeDef bus_status;	/* every transfer returns this if set */
static HAL_StatusTypeDef probe_status;	/* IsDeviceReady on a present chip */
static bool absent;		/* nothing ACKs at the address */
static uint16_t probed_addr;

static uint32_t reset_tick;
static uint32_t ready_after_ms;
static uint32_t last_poll_ms;	/* the last MEAS_CFG read, since the reset */

static uint32_t ms_since_reset(void)
{
	return stub_tick - reset_tick;	/* unsigned: safe across the wrap */
}

/* a plausible coefficient block, distinct in every field */
static const uint8_t COEFS[DPS310_COEF_LEN] = {
	0x12, 0x34, 0x56,		/* c0, c1 */
	0x78, 0x9A, 0xBC, 0xDE, 0xF0,	/* c00, c10 */
	0x11, 0x22,			/* c01 */
	0x33, 0x44,			/* c11 */
	0x55, 0x66,			/* c20 */
	0x77, 0x88,			/* c21 */
	0x99, 0xAA,			/* c30 */
};

static void chip_reset(void)
{
	absent = false;
	probed_addr = 0;
	memset(regs, 0, sizeof(regs));
	regs[DPS310_PRODUCT_ID] = DPS310_PRODUCT_ID_VALUE;
	memcpy(&regs[DPS310_COEF_BASE], COEFS, sizeof(COEFS));
	regs[DPS310_COEF_SRCE] = 0x00;
	regs[DPS310_ERRATA_PROBE] = DPS310_ERRATA_OK_BIT;	/* already fixed */
	regs[DPS310_MEAS_CFG] = DPS310_SENSOR_RDY | DPS310_COEF_RDY;
	reset_count = errata_writes = ready_delay = coef_delay = 0;
	allocs = frees = writes = reads = 0;
	dev_addr = 0;
	bus_status = HAL_OK;
	probe_status = HAL_OK;
	ready_after_ms = 0;
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
				    uint16_t address, uint16_t reg,
				    uint16_t reg_size, uint8_t *data,
				    uint16_t len, uint32_t timeout)
{
	assert(hi2c == &bus);
	assert(!(address & 1) && "the HAL takes the 7-bit address << 1");
	assert(reg_size == I2C_MEMADD_SIZE_8BIT && reg <= 0xFF);
	assert(len == 1 && "a register write is one value");
	assert(timeout > 0);
	if (absent)
		return HAL_ERROR;
	if (bus_status != HAL_OK)
		return bus_status;
	writes++;
	dev_addr = address >> 1;
	if (reg == DPS310_RESET && data[0] == DPS310_RESET_MAGIC) {
		reset_count++;
		reset_tick = stub_tick;
		return HAL_OK;
	}
	if (reg == 0x0E || reg == 0x0F || reg == 0x62)
		errata_writes++;
	regs[reg] = data[0];
	return HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t address,
				   uint16_t reg, uint16_t reg_size,
				   uint8_t *rx, uint16_t rx_len,
				   uint32_t timeout)
{
	const uint8_t first = (uint8_t)reg;

	assert(hi2c == &bus);
	assert(!(address & 1) && "the HAL takes the 7-bit address << 1");
	assert(reg_size == I2C_MEMADD_SIZE_8BIT && reg <= 0xFF);
	assert(timeout > 0);
	if (absent)
		return HAL_ERROR;
	if (bus_status != HAL_OK)
		return bus_status;
	reads++;
	dev_addr = address >> 1;
	if (reg == DPS310_MEAS_CFG)
		last_poll_ms = ms_since_reset();

	/* Optionally make ready take a few polls, as a real part does. */
	if (first == DPS310_MEAS_CFG && ready_delay > 0) {
		ready_delay--;
		rx[0] = 0;
		return HAL_OK;
	}
	/* Or take a while in real time after the soft reset. */
	if (first == DPS310_MEAS_CFG && ms_since_reset() < ready_after_ms) {
		rx[0] = 0;
		return HAL_OK;
	}
	/*
	 * Or come up in two steps, as the datasheet allows: the sensor first,
	 * the calibration later. Until COEF_RDY the coefficients read 0.
	 */
	if (first == DPS310_MEAS_CFG && coef_delay > 0) {
		coef_delay--;
		rx[0] = DPS310_SENSOR_RDY;
		return HAL_OK;
	}
	for (size_t i = 0; i < rx_len; i++) {
		unsigned int r = (first + i) & 0xFF;
		bool coef = r >= DPS310_COEF_BASE &&
			    r < DPS310_COEF_BASE + DPS310_COEF_LEN;

		rx[i] = (coef && coef_delay > 0) ? 0 : regs[r];
	}
	return HAL_OK;
}

/*
 * HAL_Delay(n) returns once HAL_GetTick() has moved n + 1. A HAL_Delay(0)
 * would still move it by one, but it means a poll with no pause, so it fails
 * loudly here.
 */
void HAL_Delay(uint32_t ms)
{
	assert(ms > 0);
	stub_tick += ms + 1;
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

static struct dps310_config base_cfg(void)
{
	struct dps310_config c = {
		.pressure_rate = DPS310_RATE_8,
		.pressure_osr = DPS310_OSR_16,
		.temperature_rate = DPS310_RATE_8,
		.temperature_osr = DPS310_OSR_2,
	};

	return c;
}

static struct baro_dev open_ok(void)
{
	struct dps310_config c = base_cfg();
	struct baro_dev baro = { 0 };

	chip_reset();
	assert(dps310_probe(&baro, &bus, &c) == 0);
	assert(baro.ops && baro.priv);
	return baro;
}

/* ------------------------------------------------------------- tests ----- */

static void test_probe(void)
{
	struct dps310_config c = base_cfg();
	struct baro_dev baro = { 0 };

	chip_reset();
	assert(dps310_probe(&baro, &bus, &c) == 0);
	assert(!strcmp(baro.name, "dps310"));
	assert(probed_addr == DPS310_ADDR_HIGH && "0 selects 0x77");
	assert(dev_addr == DPS310_ADDR_HIGH);
	assert(reset_count == 1);
	dps310_remove(&baro);
	assert(!baro.ops && !baro.priv && "remove zeroes the binding");
	assert(regs[DPS310_MEAS_CFG] == 0 && "remove leaves it idle");
	assert(allocs == 1 && frees == 1);

	chip_reset();
	c.address = DPS310_ADDR_LOW;
	assert(dps310_probe(&baro, &bus, &c) == 0);
	assert(dev_addr == DPS310_ADDR_LOW);
	dps310_remove(&baro);

	/* A wrong identity is not this part, and the log says what it saw. */
	chip_reset();
	c = base_cfg();
	regs[DPS310_PRODUCT_ID] = 0x58;		/* a BMP280 would answer this */
	assert(dps310_probe(&baro, &bus, &c) == -ENODEV);
	assert(strstr(stub_log_last, "baro: dps310: "));
	assert(strstr(stub_log_last, "product ID 0x58, expected 0x10"));
	assert(!baro.ops && !baro.priv);
	assert(allocs == 1 && frees == 1 && "nothing leaked");

	chip_reset();
	c.address = 0x60;
	assert(dps310_probe(&baro, &bus, &c) == -EINVAL);

	/* Bad rate or oversampling is refused before anything is written. */
	chip_reset();
	c = base_cfg();
	c.pressure_osr = (enum dps310_osr)8;
	assert(dps310_probe(&baro, &bus, &c) == -EINVAL);
	chip_reset();
	c = base_cfg();
	c.temperature_rate = (enum dps310_rate)8;
	assert(dps310_probe(&baro, &bus, &c) == -EINVAL);

	/* So is a config that cannot finish its measurements inside a second. */
	chip_reset();
	c = base_cfg();
	c.pressure_rate = DPS310_RATE_16;
	c.pressure_osr = DPS310_OSR_64;
	assert(dps310_probe(&baro, &bus, &c) == -EINVAL);
	assert(strstr(stub_log_last, "need 1712.0 ms of every second"));
	assert(reset_count == 0 && probed_addr == 0 && allocs == 0);

	/* Either side of the 990 ms limit: 989.6 ms is accepted, 993.6 ms not. */
	chip_reset();
	c = base_cfg();
	c.pressure_rate = DPS310_RATE_2;
	c.pressure_osr = DPS310_OSR_32;
	c.temperature_rate = DPS310_RATE_32;
	c.temperature_osr = DPS310_OSR_16;
	assert(dps310_probe(&baro, &bus, &c) == 0);
	dps310_remove(&baro);
	chip_reset();
	c.pressure_rate = DPS310_RATE_4;
	c.pressure_osr = DPS310_OSR_16;
	assert(dps310_probe(&baro, &bus, &c) == -EINVAL);

	/* The README and example config, 953.6 ms, must still start. */
	chip_reset();
	c = base_cfg();
	c.pressure_rate = DPS310_RATE_8;
	c.pressure_osr = DPS310_OSR_64;
	c.temperature_rate = DPS310_RATE_8;
	c.temperature_osr = DPS310_OSR_8;
	assert(dps310_probe(&baro, &bus, &c) == 0);
	dps310_remove(&baro);

	/*
	 * No ACK is a different failure from the wrong chip, decided by
	 * HAL_I2C_IsDeviceReady() before anything is allocated or sent.
	 */
	chip_reset();
	absent = true;
	assert(dps310_probe(&baro, &bus, &c) == -ENXIO);
	assert(probed_addr == (c.address ? c.address : DPS310_ADDR_HIGH));
	assert(allocs == 0 && writes == 0 && reads == 0);
	assert(!baro.ops && !baro.priv);

	/* A bus that is busy or hangs during the presence check is not absence. */
	chip_reset();
	probe_status = HAL_BUSY;	/* also: a handle never initialised */
	assert(dps310_probe(&baro, &bus, &c) == -EAGAIN);
	assert(allocs == 0 && writes == 0 && reads == 0);
	chip_reset();
	probe_status = HAL_TIMEOUT;
	assert(dps310_probe(&baro, &bus, &c) == -ETIMEDOUT);
	assert(allocs == 0);

	/* A NACK in the middle of a transfer is an I/O error, not absence. */
	chip_reset();
	bus_status = HAL_ERROR;
	assert(dps310_probe(&baro, &bus, &c) == -EIO);
	assert(allocs == 1 && frees == 1);
	chip_reset();
	/* Not -EBUSY: from a probe that means "already bound". */
	bus_status = HAL_BUSY;
	assert(dps310_probe(&baro, &bus, &c) == -EAGAIN);
	assert(allocs == 1 && frees == 1);
}

static void test_waits_for_ready(void)
{
	struct dps310_config c = base_cfg();
	struct baro_dev baro = { 0 };
	int32_t k[9];

	/* Coefficients read before COEF_RDY are zeros. The driver must wait. */
	chip_reset();
	ready_delay = 3;
	assert(dps310_probe(&baro, &bus, &c) == 0);
	assert(dps310_coefficients(&baro, k) == 0);
	assert(k[0] == 0x123 && "coefficients read after ready, not before");
	dps310_remove(&baro);

	/*
	 * SENSOR_RDY alone is not enough: the coefficients still read 0 until
	 * COEF_RDY follows. A probe that waited for the sensor only would
	 * unpack zeros here.
	 */
	chip_reset();
	coef_delay = 3;
	assert(dps310_probe(&baro, &bus, &c) == 0);
	assert(coef_delay == 0 && "polled until COEF_RDY too");
	assert(dps310_coefficients(&baro, k) == 0);
	assert(k[0] == 0x123 && "coefficients read after COEF_RDY, not before");
	dps310_remove(&baro);

	/* Never ready is a timeout, not a silent success with zero calibration. */
	chip_reset();
	ready_delay = 10000;
	assert(dps310_probe(&baro, &bus, &c) == -ETIMEDOUT);
	assert(!baro.ops && !baro.priv);
	assert(allocs == 1 && frees == 1);
}

static void test_ready_wait_timing(void)
{
	struct dps310_config c = base_cfg();
	struct baro_dev baro = { 0 };

	/*
	 * The wait is timed by HAL_GetTick(), not by counting polls. The clock
	 * starts just short of wrapping, so the wait straddles the wrap.
	 */
	stub_tick = (uint32_t)-6;

	chip_reset();
	ready_after_ms = 60;
	assert(dps310_probe(&baro, &bus, &c) == 0);
	dps310_remove(&baro);

	/*
	 * Readiness counts from the RESET write. The 40 ms reset wait moves the
	 * tick 41, then polls are HAL_Delay(5), 6 ticks apart, until more than
	 * 250 have passed: the last read is 252 into the wait, 293 after the
	 * reset. Ready at 293 ms still counts; 294 ms is a timeout, and the
	 * last poll is past the full 40 + 250 ms.
	 */
	chip_reset();
	ready_after_ms = 293;
	assert(dps310_probe(&baro, &bus, &c) == 0);
	dps310_remove(&baro);
	chip_reset();
	ready_after_ms = 294;
	assert(dps310_probe(&baro, &bus, &c) == -ETIMEDOUT);
	assert(last_poll_ms > 40 + 250 && "the last read is past the deadline");

	stub_tick = 0;
}

static void test_errata(void)
{
	struct dps310_config c = base_cfg();
	struct baro_dev baro = { 0 };

	/* A part that already carries the fix must be left alone. */
	chip_reset();
	regs[DPS310_ERRATA_PROBE] = DPS310_ERRATA_OK_BIT;
	assert(dps310_probe(&baro, &bus, &c) == 0);
	assert(errata_writes == 0);
	dps310_remove(&baro);

	/* A part that does not gets the five-write sequence. */
	chip_reset();
	regs[DPS310_ERRATA_PROBE] = 0x00;
	assert(dps310_probe(&baro, &bus, &c) == 0);
	assert(errata_writes == 5);
	assert(regs[0x62] == 0x02 && regs[0x0E] == 0x00 && regs[0x0F] == 0x00);
	dps310_remove(&baro);
}

static void test_coef_source_is_honoured(void)
{
	struct dps310_config c = base_cfg();
	struct baro_dev baro = { 0 };

	/*
	 * Calibration taken against the external sensor: TMP_CFG must say so,
	 * or both temperature and pressure come out wrong.
	 */
	chip_reset();
	regs[DPS310_COEF_SRCE] = DPS310_COEF_SRCE_MASK;
	assert(dps310_probe(&baro, &bus, &c) == 0);
	assert(regs[DPS310_TMP_CFG] & DPS310_TMP_EXT);
	dps310_remove(&baro);

	chip_reset();
	regs[DPS310_COEF_SRCE] = 0x00;
	assert(dps310_probe(&baro, &bus, &c) == 0);
	assert(!(regs[DPS310_TMP_CFG] & DPS310_TMP_EXT));
	/* and the rate and oversampling still made it into the same register */
	assert(((regs[DPS310_TMP_CFG] >> DPS310_RATE_SHIFT) & 0x7) ==
	       DPS310_RATE_8);
	assert((regs[DPS310_TMP_CFG] & 0x0F) == DPS310_OSR_2);
	dps310_remove(&baro);
}

static void test_shift_bit(void)
{
	struct dps310_config c = base_cfg();
	struct baro_dev baro = { 0 };

	/* 16x pressure needs the shift; 2x temperature does not. */
	chip_reset();
	assert(dps310_probe(&baro, &bus, &c) == 0);
	assert(regs[DPS310_CFG_REG] & DPS310_PRS_SHIFT_EN);
	assert(!(regs[DPS310_CFG_REG] & DPS310_TMP_SHIFT_EN));
	dps310_remove(&baro);

	/* Both low: neither shift bit. */
	chip_reset();
	c.pressure_osr = DPS310_OSR_8;
	c.temperature_osr = DPS310_OSR_1;
	assert(dps310_probe(&baro, &bus, &c) == 0);
	assert(regs[DPS310_CFG_REG] == 0);
	dps310_remove(&baro);

	/* Both high: both bits. At 1 Hz, so it fits the one-second budget. */
	chip_reset();
	c.pressure_rate = DPS310_RATE_1;
	c.temperature_rate = DPS310_RATE_1;
	c.pressure_osr = DPS310_OSR_128;
	c.temperature_osr = DPS310_OSR_64;
	assert(dps310_probe(&baro, &bus, &c) == 0);
	assert((regs[DPS310_CFG_REG] &
		(DPS310_PRS_SHIFT_EN | DPS310_TMP_SHIFT_EN)) ==
	       (DPS310_PRS_SHIFT_EN | DPS310_TMP_SHIFT_EN));
	dps310_remove(&baro);

	/* Against the datasheet, not the macro: P_SHIFT is bit 2, bit 4 is INT_PRS. */
	chip_reset();
	c = base_cfg();
	c.pressure_osr = DPS310_OSR_64;
	assert(dps310_probe(&baro, &bus, &c) == 0);
	assert(regs[DPS310_CFG_REG] == 0x04);
	assert(regs[DPS310_MEAS_CFG] == 0x07 && "background, both sensors");
	dps310_remove(&baro);
}

static void test_read(void)
{
	struct baro_dev baro = open_ok();
	struct baro_sample s;
	struct dps310_coefs c;
	int32_t p, t, k[9];
	float kt, kp, tsc, psc;

	/* Results are 24-bit, MSB first, and adjacent. */
	regs[0x00] = 0x00; regs[0x01] = 0x10; regs[0x02] = 0x00; /* pressure */
	regs[0x03] = 0xFF; regs[0x04] = 0xFF; regs[0x05] = 0xFF; /* temp = -1 */

	assert(dps310_read_raw(&baro, &p, &t) == 0);
	assert(p == 0x1000);
	assert(t == -1);

	/* The compensated values must match the maths header applied by hand. */
	stub_tick = 777;
	assert(baro_read(&baro, &s) == 0);
	assert(s.t_us == 777000);
	assert(dps310_coefficients(&baro, k) == 0);
	c.c0 = k[0]; c.c1 = k[1]; c.c00 = k[2]; c.c10 = k[3];
	c.c01 = k[4]; c.c11 = k[5]; c.c20 = k[6]; c.c21 = k[7]; c.c30 = k[8];

	kt = dps310_scale_factor(DPS310_OSR_2);
	kp = dps310_scale_factor(DPS310_OSR_16);
	tsc = (float)t / kt;
	psc = (float)p / kp;
	assert(fabs(s.temp_c - dps310_temperature_c(&c, tsc)) < 1e-3);
	assert(fabs(s.pressure_pa - dps310_pressure_pa(&c, psc, tsc)) < 1e-1);

	/* A bus error fails the read and leaves the sample alone. */
	bus_status = HAL_TIMEOUT;
	s.t_us = 5;
	assert(baro_read(&baro, &s) == -ETIMEDOUT);
	assert(s.t_us == 5);
	bus_status = HAL_OK;

	dps310_remove(&baro);
}

static void test_data_ready(void)
{
	struct baro_dev baro = open_ok();
	bool ready = true;

	/* Both results are needed, not just one. */
	regs[DPS310_MEAS_CFG] = DPS310_PRS_RDY;
	assert(dps310_data_ready(&baro, &ready) == 0 && !ready);
	regs[DPS310_MEAS_CFG] = DPS310_TMP_RDY;
	assert(dps310_data_ready(&baro, &ready) == 0 && !ready);
	regs[DPS310_MEAS_CFG] = DPS310_PRS_RDY | DPS310_TMP_RDY;
	assert(dps310_data_ready(&baro, &ready) == 0 && ready);
	dps310_remove(&baro);
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
	struct dps310_config c = base_cfg();
	struct baro_dev baro = { 0 };
	int32_t p, t, k[9];
	bool b;
	uint8_t v;

	chip_reset();
	assert(dps310_probe(NULL, &bus, &c) == -EINVAL);
	assert(dps310_probe(&baro, &bus, NULL) == -EINVAL);
	assert(dps310_probe(&baro, NULL, &c) == -EINVAL);
	assert(probed_addr == 0 && allocs == 0 &&
	       "refused before calloc and the bus");
	assert(!baro.ops && !baro.priv);
	assert(dps310_read_raw(NULL, &p, &t) == -EINVAL);
	assert(dps310_data_ready(NULL, &b) == -EINVAL);
	assert(dps310_coefficients(NULL, k) == -EINVAL);
	assert(dps310_reg_read(NULL, 0, &v, 1) == -EINVAL);
	assert(dps310_reg_write(NULL, 0, 0) == -EINVAL);
	dps310_remove(NULL);

	/* Unprobed: every extra refuses it. */
	assert(dps310_read_raw(&baro, &p, &t) == -EINVAL);
	assert(dps310_reg_write(&baro, 0, 0) == -EINVAL);
	dps310_remove(&baro);

	/* Another chip's baro_dev is refused, never cast to ours. */
	chip_reset();
	assert(dps310_read_raw(&other, &p, &t) == -EINVAL);
	assert(dps310_data_ready(&other, &b) == -EINVAL);
	assert(dps310_coefficients(&other, k) == -EINVAL);
	assert(dps310_reg_read(&other, 0, &v, 1) == -EINVAL);
	assert(dps310_reg_write(&other, 0, 0) == -EINVAL);
	dps310_remove(&other);
	assert(other.ops == &other_ops && other.priv == regs &&
	       "not ours to remove");
	assert(reads == 0 && writes == 0 && "and no bus traffic for it");
	assert(frees == 0);

	baro = open_ok();
	assert(dps310_read_raw(&baro, NULL, &t) == -EINVAL);
	assert(dps310_read_raw(&baro, &p, NULL) == -EINVAL);
	assert(dps310_data_ready(&baro, NULL) == -EINVAL);
	assert(dps310_coefficients(&baro, NULL) == -EINVAL);
	assert(dps310_reg_read(&baro, 0, &v, 0) == -EINVAL);
	assert(dps310_reg_read(&baro, 0, NULL, 1) == -EINVAL);
	dps310_remove(&baro);
}

static void test_second_probe_is_refused(void)
{
	static const struct baro_ops other_ops = { .read = other_read };
	struct baro_dev other = { .name = "other", .ops = &other_ops };
	struct dps310_config c = base_cfg();
	struct baro_dev baro = open_ok();
	void *priv = baro.priv;
	int w = writes, r = reads, n = allocs;
	int32_t k[9];

	probed_addr = 0;
	assert(dps310_probe(&baro, &bus, &c) == -EBUSY);
	assert(probed_addr == 0 && writes == w && reads == r &&
	       "no bus traffic");
	assert(allocs == n && "no allocation");
	assert(baro.priv == priv && dps310_coefficients(&baro, k) == 0 &&
	       "first binding intact");
	dps310_remove(&baro);

	/* Bound by another driver: refused too, and left exactly as it was. */
	assert(dps310_probe(&other, &bus, &c) == -EBUSY);
	assert(!strcmp(other.name, "other") && other.ops == &other_ops);

	/* A removed device can be probed afresh. */
	chip_reset();
	assert(dps310_probe(&baro, &bus, &c) == 0);
	dps310_remove(&baro);
}

int main(void)
{
	test_probe();
	test_waits_for_ready();
	test_ready_wait_timing();
	test_errata();
	test_coef_source_is_honoured();
	test_shift_bit();
	test_read();
	test_data_ready();
	test_null_and_foreign_arguments();
	test_second_probe_is_refused();
	printf("all dps310 driver checks passed\n");
	return 0;
}
