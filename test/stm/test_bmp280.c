/*
 * Host-side check of the driver against a fake BMP280 behind a fake HAL. No
 * hardware, no ARM toolchain -- see test/Makefile.
 *
 * The fake is a 256-byte register file plus the two things on this chip that
 * take time: the start-up after a soft reset, during which the trim registers
 * read zeros, and a forced measurement, during which the result registers
 * hold whatever they held before -- 0x80000 on a part that has just been
 * reset. It charges the datasheet *maximum* times, computed from the formula
 * rather than from the driver's rounded table, so the driver cannot pass by
 * agreeing with itself.
 *
 * HAL_Delay(n) waits n + 1 ticks, the first of which may be partial, so all
 * it guarantees is n whole milliseconds. That is all the fake credits. The
 * fake HAL also insists on the HAL's 8-bit address form, the 7-bit address
 * shifted left one.
 */
#include <assert.h>
#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "stm32_hal_stub.h"

#include "baro.h"
#include "bmp280.h"
#include "bmp280_math.h"
#include "bmp280_regs.h"

/*
 * The same plausible trim set test_bmp280_math.c uses, as the chip stores it:
 * dig_T1..T3, dig_P1..P9, little-endian.
 */
static const uint16_t CAL_WORDS[12] = {
	27504, 26435, (uint16_t)-1000, 36477, (uint16_t)-10685, 3024,
	2855, 140, (uint16_t)-7, 15500, (uint16_t)-14600, 6000,
};

/*
 * Raw values that compensate to 25.08 C and 100653.25 Pa -- pinned in
 * test_bmp280_math.c, and self-generated there too.
 */
#define RAW_T		519888
#define RAW_P		415148
#define EXPECT_C	25.08f
#define EXPECT_PA	100653.254f

#define RESET_US	2000	/* start-up time, 2 ms */

static I2C_HandleTypeDef bus;
static uint8_t regs[256];
static uint8_t chip_id;
static uint16_t cal[12];
static int32_t raw_t, raw_p;	/* what the next finished measurement yields */
static long need_us, elapsed_us;
static enum { P_NONE, P_RESET, P_MEAS } pending;
static uint16_t dev_addr;	/* 7-bit, of the last transfer */
static int writes, reads, resets, bursts;
static uint8_t last_ctrl, last_config;
static HAL_StatusTypeDef bus_status;	/* every transfer returns this if set */
static HAL_StatusTypeDef probe_status;	/* IsDeviceReady on a present chip */
static bool absent;		/* nothing ACKs at the address */
static uint16_t probed_addr;
static int allocs, frees;

static void put20(uint8_t *p, int32_t v)
{
	p[0] = (uint8_t)(v >> 12);
	p[1] = (uint8_t)(v >> 4);
	p[2] = (uint8_t)((v & 0x0F) << 4);
}

/*
 * A soft reset, as the chip does it: results back to 0x80000, ctrl_meas and
 * config cleared, trim reading zeros until the NVM copy finishes.
 */
static void chip_soft_reset(void)
{
	memset(&regs[BMP280_REG_CALIB], 0, BMP280_CALIB_LEN);
	regs[BMP280_REG_CTRL_MEAS] = 0;
	regs[BMP280_REG_CONFIG] = 0;
	regs[BMP280_REG_STATUS] = BMP280_STATUS_IM_UPDATE;
	put20(&regs[BMP280_REG_PRESS_MSB], BMP280_RAW_RESET);
	put20(&regs[BMP280_REG_PRESS_MSB + 3], BMP280_RAW_RESET);
	pending = P_RESET;
	need_us = RESET_US;
	elapsed_us = 0;
	resets++;
}

static void chip_power_on(void)
{
	absent = false;
	probed_addr = 0;
	dev_addr = 0;
	memset(regs, 0, sizeof(regs));
	chip_id = BMP280_CHIP_ID;
	memcpy(cal, CAL_WORDS, sizeof(cal));
	raw_t = RAW_T;
	raw_p = RAW_P;
	writes = reads = resets = bursts = 0;
	allocs = frees = 0;
	last_ctrl = last_config = 0xFF;
	bus_status = HAL_OK;
	probe_status = HAL_OK;
	chip_soft_reset();
	resets = 0;
}

/* Apply whatever the chip was busy with, once enough time has gone by. */
static void settle(void)
{
	if (pending == P_NONE || elapsed_us < need_us)
		return;

	if (pending == P_RESET) {
		for (int i = 0; i < 12; i++) {
			regs[BMP280_REG_CALIB + 2 * i] = (uint8_t)cal[i];
			regs[BMP280_REG_CALIB + 2 * i + 1] =
				(uint8_t)(cal[i] >> 8);
		}
		regs[BMP280_REG_STATUS] &= (uint8_t)~BMP280_STATUS_IM_UPDATE;
	} else {
		put20(&regs[BMP280_REG_PRESS_MSB], raw_p);
		put20(&regs[BMP280_REG_PRESS_MSB + 3], raw_t);
		regs[BMP280_REG_STATUS] &= (uint8_t)~BMP280_STATUS_MEASURING;
		/* forced returns to sleep */
		regs[BMP280_REG_CTRL_MEAS] &= (uint8_t)~0x03;
	}
	pending = P_NONE;
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
	uint8_t val;

	assert(hi2c == &bus);
	assert(!(address & 1) && "the HAL takes the 7-bit address << 1");
	assert(reg_size == I2C_MEMADD_SIZE_8BIT && reg <= 0xFF);
	assert(len == 1 && "a write is the register address then one byte");
	assert(timeout > 0);
	if (absent)
		return HAL_ERROR;
	if (bus_status != HAL_OK)
		return bus_status;
	val = data[0];
	writes++;
	dev_addr = address >> 1;

	if (reg == BMP280_REG_RESET) {
		if (val == BMP280_RESET_WORD)
			chip_soft_reset();	/* any other value is ignored */
		return HAL_OK;
	}
	if (reg == BMP280_REG_CONFIG) {
		last_config = val;
		regs[reg] = val;
		return HAL_OK;
	}
	assert(reg == BMP280_REG_CTRL_MEAS && "the driver writes nothing else");
	last_ctrl = val;
	regs[reg] = val;

	if ((val & 0x03) == BMP280_MODE_FORCED || (val & 0x03) == 0x02) {
		long ot, op;

		assert(((val >> 5) & 7) != 0 && ((val >> 2) & 7) != 0 &&
		       "a skipped field never produces a result");
		ot = 1L << (((val >> 5) & 7) - 1);
		op = 1L << (((val >> 2) & 7) - 1);
		/* the datasheet maximum */
		need_us = 1250 + 2300 * ot + 2300 * op + 575;
		elapsed_us = 0;
		pending = P_MEAS;
		regs[BMP280_REG_STATUS] |= BMP280_STATUS_MEASURING;
	}
	return HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t address,
				   uint16_t reg, uint16_t reg_size,
				   uint8_t *data, uint16_t len,
				   uint32_t timeout)
{
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
	settle();

	if (reg == BMP280_REG_ID) {
		assert(len == 1);
		data[0] = chip_id;
		return HAL_OK;
	}
	if (reg == BMP280_REG_PRESS_MSB) {
		assert(len == BMP280_RESULT_LEN && "both results in one burst");
		bursts++;
	}
	assert((size_t)reg + len <= sizeof(regs));
	memcpy(data, &regs[reg], len);	/* reads auto-increment */
	return HAL_OK;
}

/* HAL_Delay(n): n + 1 ticks, of which only n whole milliseconds are sure. */
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

static struct baro_dev open_ok(const struct bmp280_config *cfg)
{
	struct baro_dev baro = { 0 };

	chip_power_on();
	assert(bmp280_probe(&baro, &bus, cfg) == 0);
	assert(baro.ops && baro.priv);
	return baro;
}

/* ------------------------------------------------------------- tests ----- */

static void test_probe_defaults(void)
{
	struct baro_dev baro = open_ok(NULL);

	assert(!strcmp(baro.name, "bmp280"));
	assert(probed_addr == BMP280_ADDR_SDO_LOW && "NULL cfg selects 0x76");
	assert(dev_addr == BMP280_ADDR_SDO_LOW);
	assert(resets == 1 && "probe soft-resets once");
	assert(last_config == 0xFF &&
	       "config untouched: the reset already zeroed it");
	assert(regs[BMP280_REG_CONFIG] == 0 && "IIR off, spi3w off");
	assert(regs[BMP280_REG_CTRL_MEAS] == 0 && "the chip is left asleep");
	bmp280_remove(&baro);
	assert(allocs == 1 && frees == 1);
	assert(!baro.ops && !baro.priv && "remove zeroes the binding");
}

static void test_wrong_chip_id(void)
{
	struct baro_dev baro = { 0 };

	/*
	 * A BME280 answers on the same addresses, with the same trim words and
	 * the same pressure and temperature maths -- it would read correctly.
	 * Refused because this driver only claims the BMP280.
	 */
	chip_power_on();
	chip_id = BME280_CHIP_ID;
	assert(bmp280_probe(&baro, &bus, NULL) == -ENODEV);
	assert(strstr(stub_log_last, "baro: bmp280: "));
	assert(strstr(stub_log_last, "that is a BME280"));
	assert(resets == 0 && "rejected before anything is written");
	assert(allocs == 1 && frees == 1 && "nothing leaked");
	assert(!baro.ops && !baro.priv && "a failed probe leaves it unbound");

	/* Anything else, including the 0x00 or 0xFF of a floating bus. */
	chip_power_on();
	chip_id = 0x00;
	assert(bmp280_probe(&baro, &bus, NULL) == -ENODEV);
	chip_power_on();
	chip_id = 0xFF;
	assert(bmp280_probe(&baro, &bus, NULL) == -ENODEV);
	assert(allocs == frees);
}

static void test_config(void)
{
	struct bmp280_config c = { .address = BMP280_ADDR_SDO_HIGH,
				   .osr = BMP280_OSR_ULTRA_LOW };
	struct baro_dev baro = open_ok(&c);
	struct baro_sample s;

	assert(probed_addr == BMP280_ADDR_SDO_HIGH);
	assert(dev_addr == BMP280_ADDR_SDO_HIGH);
	assert(baro_read(&baro, &s) == 0);
	assert(last_ctrl ==
	       bmp280_ctrl_meas(BMP280_OSR_ULTRA_LOW, BMP280_MODE_FORCED));
	assert(last_ctrl == 0x25 && "osrs_t x1, osrs_p x1, forced");
	bmp280_remove(&baro);

	/* Refused before anything touches the bus, and nothing allocated. */
	chip_power_on();
	c.address = 0x60;
	assert(bmp280_probe(&baro, &bus, &c) == -EINVAL);
	c.address = 0;
	c.osr = (enum bmp280_osr)5;
	assert(bmp280_probe(&baro, &bus, &c) == -EINVAL);
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
	assert(last_ctrl == 0x55 && "ultra high resolution, forced");
	assert(fabsf(s.temp_c - EXPECT_C) < 0.005f);
	assert(fabsf(s.pressure_pa - EXPECT_PA) < 0.01f);
	assert(s.t_us == 5555000);

	/* A second read triggers again: forced mode measures once and sleeps. */
	assert(baro_read(&baro, &s) == 0);
	assert(writes == 2 && bursts == 2);
	bmp280_remove(&baro);
}

static void test_waits_long_enough_at_every_osr(void)
{
	/*
	 * HAL_Delay() only takes whole milliseconds. Each wait must still cover
	 * the unrounded datasheet maximum, or the fake serves the stale 0x80000
	 * the reset left behind and the read fails.
	 */
	for (int osr = BMP280_OSR_ULTRA_HIGH; osr <= BMP280_OSR_ULTRA_LOW;
	     osr++) {
		struct bmp280_config c = { .osr = (enum bmp280_osr)osr };
		struct baro_dev baro = open_ok(&c);
		struct baro_sample s;

		assert(baro_read(&baro, &s) == 0);
		assert(fabsf(s.pressure_pa - EXPECT_PA) < 0.01f);
		assert(pending == P_NONE && elapsed_us >= need_us &&
		       "waited it out");
		bmp280_remove(&baro);
	}
}

static void test_reset_wait(void)
{
	/*
	 * The trim image reads zeros until the start-up time is up. Probe must
	 * not read it early -- zeros would divide by zero in the pressure path.
	 */
	struct baro_dev baro = open_ok(NULL);

	assert(pending == P_NONE &&
	       "the NVM copy had finished before the trim read");
	bmp280_remove(&baro);
}

static void test_bad_trim(void)
{
	struct baro_dev baro = { 0 };

	/* A bus that reads nothing but zeros. */
	chip_power_on();
	memset(cal, 0, sizeof(cal));
	assert(bmp280_probe(&baro, &bus, NULL) == -EIO);
	assert(allocs == 1 && frees == 1);
	assert(!baro.ops && !baro.priv);

	/* dig_P1 = 0 alone: the divide-by-zero in the compensation. */
	chip_power_on();
	cal[3] = 0;
	assert(bmp280_probe(&baro, &bus, NULL) == -EIO);
	assert(allocs == frees);

	/* dig_T1 = 0 alone. */
	chip_power_on();
	cal[0] = 0;
	assert(bmp280_probe(&baro, &bus, NULL) == -EIO);
	assert(allocs == frees);

	/* A signed word reading zero is legal and must not be rejected. */
	chip_power_on();
	cal[1] = 0;
	assert(bmp280_probe(&baro, &bus, NULL) == 0);
	bmp280_remove(&baro);
}

static void test_raw_reset_value(void)
{
	struct baro_dev baro = open_ok(NULL);
	struct baro_sample s = { .pressure_pa = 1.0f, .temp_c = 1.0f,
				 .t_us = 3 };

	/* Both results at 0x80000 is a chip that has never measured. */
	raw_p = raw_t = BMP280_RAW_RESET;
	assert(baro_read(&baro, &s) == -EIO);
	assert(s.pressure_pa == 1.0f && s.temp_c == 1.0f && s.t_us == 3 &&
	       "sample untouched");

	/* One of them alone is an ordinary reading and must go through. */
	raw_p = RAW_P;
	assert(baro_read(&baro, &s) == 0);
	raw_p = BMP280_RAW_RESET;
	raw_t = RAW_T;
	assert(baro_read(&baro, &s) == 0);
	bmp280_remove(&baro);
}

static void test_pressure_out_of_range(void)
{
	struct baro_dev baro = open_ok(NULL);
	struct baro_sample s = { .pressure_pa = 1.0f, .temp_c = 1.0f };

	/*
	 * Raw pressure at the top of its range compensates negative, which the
	 * datasheet's cast turns into about 16 MPa. Reported as an error, not
	 * as a number.
	 */
	raw_p = 0xFFFFF;
	assert(baro_read(&baro, &s) == -ERANGE);
	assert(s.pressure_pa == 1.0f && "sample untouched");
	bmp280_remove(&baro);
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
	assert(bmp280_probe(&baro, &bus, NULL) == -ENXIO);
	assert(probed_addr == BMP280_ADDR_SDO_LOW);
	assert(allocs == 0 && writes == 0 && reads == 0);
	assert(!baro.ops && !baro.priv);

	/* A bus that is busy or hangs during the presence check is not absence. */
	chip_power_on();
	probe_status = HAL_BUSY;	/* also: a handle never initialised */
	assert(bmp280_probe(&baro, &bus, NULL) == -EAGAIN);
	assert(allocs == 0 && writes == 0 && reads == 0);
	chip_power_on();
	probe_status = HAL_TIMEOUT;
	assert(bmp280_probe(&baro, &bus, NULL) == -ETIMEDOUT);
	assert(allocs == 0);

	/* A NACK in the middle of a transfer is an I/O error, not absence. */
	chip_power_on();
	bus_status = HAL_ERROR;
	assert(bmp280_probe(&baro, &bus, NULL) == -EIO);
	assert(allocs == 1 && frees == 1);
	chip_power_on();
	/* Not -EBUSY: from a probe that means "already bound". */
	bus_status = HAL_BUSY;
	assert(bmp280_probe(&baro, &bus, NULL) == -EAGAIN);
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
	bmp280_remove(&baro);
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
	assert(bmp280_probe(NULL, &bus, NULL) == -EINVAL);
	assert(bmp280_probe(&baro, NULL, NULL) == -EINVAL);
	assert(allocs == 0 && writes == 0 && probed_addr == 0);
	bmp280_remove(NULL);
	bmp280_remove(&baro);			/* unprobed: a no-op */

	/* Another chip's baro_dev is not ours to remove. */
	bmp280_remove(&other);
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
	assert(bmp280_probe(&baro, &bus, NULL) == -EBUSY);
	assert(probed_addr == 0 && writes == w && reads == r &&
	       "no bus traffic");
	assert(allocs == n && "no allocation");
	assert(baro.priv == priv && baro_read(&baro, &s) == 0 &&
	       "first binding intact");
	bmp280_remove(&baro);

	/* Bound by another driver: refused too, and left exactly as it was. */
	assert(bmp280_probe(&other, &bus, NULL) == -EBUSY);
	assert(!strcmp(other.name, "other") && other.ops == &other_ops);

	/* A removed device can be probed afresh. */
	chip_power_on();
	assert(bmp280_probe(&baro, &bus, NULL) == 0);
	bmp280_remove(&baro);
}

int main(void)
{
	test_probe_defaults();
	test_wrong_chip_id();
	test_config();
	test_read();
	test_waits_long_enough_at_every_osr();
	test_reset_wait();
	test_bad_trim();
	test_raw_reset_value();
	test_pressure_out_of_range();
	test_bus_error();
	test_null_and_foreign_arguments();
	test_second_probe_is_refused();
	printf("all bmp280 driver checks passed\n");
	return 0;
}
