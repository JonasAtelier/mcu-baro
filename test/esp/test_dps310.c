/*
 * Host-side check of the driver against a fake DPS310. No hardware, no ESP
 * toolchain -- see test/Makefile.
 *
 * The fake models the two sequencing rules that matter: the coefficients are
 * meaningless until COEF_RDY is set, and the temperature source has to match
 * where the calibration came from. Both produce readings that look fine.
 *
 * The clock is simulated: only vTaskDelay moves it. Most tests run at a
 * 1 kHz tick; test_slow_tick() runs at IDF's default 100 Hz, where the ready
 * wait used to burn its tries in no time (fix ade066d in esp-dps310-baro).
 */
#include <assert.h>
#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "baro.h"
#include "dps310.h"
#include "dps310_math.h"
#include "dps310_regs.h"

static uint8_t regs[256];
static uint16_t dev_addr;
static int reset_count, errata_writes, ready_delay;
/* MEAS_CFG polls that report SENSOR_RDY alone, coefficients reading 0 */
static int coef_delay;
static int added, removed, writes, reads;
static esp_err_t bus_error;	/* every transfer fails with this if set */
static bool absent;		/* nothing ACKs at the address */
static uint16_t probed_addr;

#define BUS ((i2c_master_bus_handle_t)(void *)regs)

static TickType_t now, reset_tick;
static unsigned int ready_after_ms;

static unsigned int ms_since_reset(void)
{
	return (TickType_t)(now - reset_tick) * 1000u / stub_tick_hz;
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
	added = removed = writes = reads = 0;
	bus_error = ESP_OK;
	ready_after_ms = 0;
}

esp_err_t i2c_master_bus_add_device(i2c_master_bus_handle_t bus,
				    const i2c_device_config_t *cfg,
				    i2c_master_dev_handle_t *out)
{
	(void)bus;
	assert(cfg->dev_addr_length == I2C_ADDR_BIT_LEN_7);
	assert(cfg->scl_speed_hz > 0 && cfg->scl_speed_hz <= 400000);
	dev_addr = cfg->device_address;
	added++;
	*out = (i2c_master_dev_handle_t)(void *)regs;
	return ESP_OK;
}

/*
 * The address-only probe. IDF 5.5 reports a NACK here as ESP_ERR_NOT_FOUND;
 * a NACKed transfer is ESP_ERR_INVALID_STATE instead, which the transfer
 * fakes return for an absent chip.
 */
esp_err_t i2c_master_probe(i2c_master_bus_handle_t bus, uint16_t address,
			   int xfer_timeout_ms)
{
	(void)bus;
	(void)xfer_timeout_ms;
	probed_addr = address;
	return absent ? ESP_ERR_NOT_FOUND : ESP_OK;
}

esp_err_t i2c_master_bus_rm_device(i2c_master_dev_handle_t d)
{
	(void)d;
	removed++;
	return ESP_OK;
}

esp_err_t i2c_master_transmit(i2c_master_dev_handle_t d, const uint8_t *data,
			      size_t len, int t)
{
	(void)d;
	(void)t;
	assert(len == 2);
	if (absent)
		return ESP_ERR_INVALID_STATE;
	if (bus_error != ESP_OK)
		return bus_error;
	writes++;
	if (data[0] == DPS310_RESET && data[1] == DPS310_RESET_MAGIC) {
		reset_count++;
		reset_tick = now;
		return ESP_OK;
	}
	if (data[0] == 0x0E || data[0] == 0x0F || data[0] == 0x62)
		errata_writes++;
	regs[data[0]] = data[1];
	return ESP_OK;
}

esp_err_t i2c_master_transmit_receive(i2c_master_dev_handle_t d,
				      const uint8_t *tx, size_t tx_len,
				      uint8_t *rx, size_t rx_len, int t)
{
	(void)d;
	(void)t;
	assert(tx_len == 1);
	if (absent)
		return ESP_ERR_INVALID_STATE;
	if (bus_error != ESP_OK)
		return bus_error;
	reads++;

	/* Optionally make ready take a few polls, as a real part does. */
	if (tx[0] == DPS310_MEAS_CFG && ready_delay > 0) {
		ready_delay--;
		rx[0] = 0;
		return ESP_OK;
	}
	/* Or take a while in real time after the soft reset. */
	if (tx[0] == DPS310_MEAS_CFG && ms_since_reset() < ready_after_ms) {
		rx[0] = 0;
		return ESP_OK;
	}
	/*
	 * Or come up in two steps, as the datasheet allows: the sensor first,
	 * the calibration later. Until COEF_RDY the coefficients read 0.
	 */
	if (tx[0] == DPS310_MEAS_CFG && coef_delay > 0) {
		coef_delay--;
		rx[0] = DPS310_SENSOR_RDY;
		return ESP_OK;
	}
	for (size_t i = 0; i < rx_len; i++) {
		unsigned int reg = (tx[0] + i) & 0xFF;
		bool coef = reg >= DPS310_COEF_BASE &&
			    reg < DPS310_COEF_BASE + DPS310_COEF_LEN;

		rx[i] = (coef && coef_delay > 0) ? 0 : regs[reg];
	}
	return ESP_OK;
}

/*
 * Real FreeRTOS takes 0 as a yield; here it would stop the clock and hang a
 * time-bounded wait, so a missing one-tick floor fails loudly instead.
 */
void vTaskDelay(TickType_t t)
{
	assert(t > 0);
	now += t;
}

TickType_t xTaskGetTickCount(void)
{
	return now;
}

/* Counts the drivers' calloc() calls; linked with -Wl,--wrap=calloc. */
static int allocs;
void *__real_calloc(size_t n, size_t size);
void *__wrap_calloc(size_t n, size_t size)
{
	allocs++;
	return __real_calloc(n, size);
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
	assert(dps310_probe(&baro, BUS, &c) == 0);
	assert(baro.ops && baro.priv);
	return baro;
}

/* ------------------------------------------------------------- tests ----- */

static void test_probe(void)
{
	struct dps310_config c = base_cfg();
	struct baro_dev baro = { 0 };

	chip_reset();
	assert(dps310_probe(&baro, BUS, &c) == 0);
	assert(!strcmp(baro.name, "dps310"));
	assert(dev_addr == DPS310_ADDR_HIGH && "0 selects 0x77");
	assert(reset_count == 1);
	dps310_remove(&baro);
	assert(!baro.ops && !baro.priv && "remove zeroes the binding");
	assert(regs[DPS310_MEAS_CFG] == 0 && "remove leaves it idle");
	assert(added == 1 && removed == 1);

	chip_reset();
	c.address = DPS310_ADDR_LOW;
	assert(dps310_probe(&baro, BUS, &c) == 0);
	assert(dev_addr == DPS310_ADDR_LOW);
	dps310_remove(&baro);

	/* A wrong identity is not this part, and the log says what it saw. */
	chip_reset();
	c = base_cfg();
	regs[DPS310_PRODUCT_ID] = 0x58;		/* a BMP280 would answer this */
	assert(dps310_probe(&baro, BUS, &c) == -ENODEV);
	assert(strstr(stub_log_last, "E baro: dps310: "));
	assert(strstr(stub_log_last, "product ID 0x58, expected 0x10"));
	assert(!baro.ops && !baro.priv);
	assert(added == 1 && removed == 1 && "nothing left on the bus");

	chip_reset();
	c.address = 0x60;
	assert(dps310_probe(&baro, BUS, &c) == -EINVAL);

	/* Bad rate or oversampling is refused before anything is written. */
	chip_reset();
	c = base_cfg();
	c.pressure_osr = (enum dps310_osr)8;
	assert(dps310_probe(&baro, BUS, &c) == -EINVAL);
	chip_reset();
	c = base_cfg();
	c.temperature_rate = (enum dps310_rate)8;
	assert(dps310_probe(&baro, BUS, &c) == -EINVAL);

	/* So is a config that cannot finish its measurements inside a second. */
	chip_reset();
	c = base_cfg();
	c.pressure_rate = DPS310_RATE_16;
	c.pressure_osr = DPS310_OSR_64;
	allocs = 0;
	assert(dps310_probe(&baro, BUS, &c) == -EINVAL);
	assert(strstr(stub_log_last, "need 1712.0 ms of every second"));
	assert(reset_count == 0 && added == 0 && allocs == 0);

	/* Either side of the 990 ms limit: 989.6 ms is accepted, 993.6 ms not. */
	chip_reset();
	c = base_cfg();
	c.pressure_rate = DPS310_RATE_2;
	c.pressure_osr = DPS310_OSR_32;
	c.temperature_rate = DPS310_RATE_32;
	c.temperature_osr = DPS310_OSR_16;
	assert(dps310_probe(&baro, BUS, &c) == 0);
	dps310_remove(&baro);
	chip_reset();
	c.pressure_rate = DPS310_RATE_4;
	c.pressure_osr = DPS310_OSR_16;
	assert(dps310_probe(&baro, BUS, &c) == -EINVAL);

	/* The README and example config, 953.6 ms, must still start. */
	chip_reset();
	c = base_cfg();
	c.pressure_rate = DPS310_RATE_8;
	c.pressure_osr = DPS310_OSR_64;
	c.temperature_rate = DPS310_RATE_8;
	c.temperature_osr = DPS310_OSR_8;
	assert(dps310_probe(&baro, BUS, &c) == 0);
	dps310_remove(&baro);

	/*
	 * No ACK is a different failure from the wrong chip, decided by
	 * i2c_master_probe() before anything is allocated or added. A NACK in
	 * the middle of a transfer (IDF 5.5: ESP_ERR_INVALID_STATE) is an I/O
	 * error, not absence.
	 */
	chip_reset();
	absent = true;
	allocs = 0;
	assert(dps310_probe(&baro, BUS, &c) == -ENXIO);
	assert(probed_addr == (c.address ? c.address : DPS310_ADDR_HIGH));
	assert(allocs == 0 && added == 0 && removed == 0);
	assert(!baro.ops && !baro.priv);
	chip_reset();
	bus_error = ESP_ERR_INVALID_STATE;
	assert(dps310_probe(&baro, BUS, &c) == -EIO);
	assert(added == 1 && removed == 1);
}

static void test_waits_for_ready(void)
{
	struct dps310_config c = base_cfg();
	struct baro_dev baro = { 0 };
	int32_t k[9];

	/* Coefficients read before COEF_RDY are zeros. The driver must wait. */
	chip_reset();
	ready_delay = 3;
	assert(dps310_probe(&baro, BUS, &c) == 0);
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
	assert(dps310_probe(&baro, BUS, &c) == 0);
	assert(coef_delay == 0 && "polled until COEF_RDY too");
	assert(dps310_coefficients(&baro, k) == 0);
	assert(k[0] == 0x123 && "coefficients read after COEF_RDY, not before");
	dps310_remove(&baro);

	/* Never ready is a timeout, not a silent success with zero calibration. */
	chip_reset();
	ready_delay = 10000;
	assert(dps310_probe(&baro, BUS, &c) == -ETIMEDOUT);
	assert(!baro.ops && !baro.priv);
	assert(added == 1 && removed == 1);
}

static void test_slow_tick(void)
{
	struct dps310_config c = base_cfg();
	struct baro_dev baro = { 0 };

	/*
	 * ESP-IDF defaults to a 100 Hz tick, where pdMS_TO_TICKS(5) is 0 and a
	 * poll that only yields burns its tries in no time at all. The clock
	 * starts just short of wrapping, so the wait straddles the wrap.
	 */
	stub_tick_hz = 100;
	now = (TickType_t)-6;

	chip_reset();
	ready_after_ms = 60;
	assert(dps310_probe(&baro, BUS, &c) == 0);
	dps310_remove(&baro);

	/*
	 * Readiness counts from the RESET write. The 40 ms reset wait is 4 + 1
	 * ticks, then the limit is 25 ticks for 250 ms plus 1 for the partial
	 * first tick: 26 x 10 ms, so the last read lands at 50 + 260 = 310 ms.
	 * Ready at 310 ms still counts; 320 ms is a timeout. One tick less on
	 * either +1 flips the first, one more flips the second.
	 */
	chip_reset();
	ready_after_ms = 310;
	assert(dps310_probe(&baro, BUS, &c) == 0);
	dps310_remove(&baro);
	chip_reset();
	ready_after_ms = 320;
	assert(dps310_probe(&baro, BUS, &c) == -ETIMEDOUT);

	stub_tick_hz = 1000;
}

static void test_errata(void)
{
	struct dps310_config c = base_cfg();
	struct baro_dev baro = { 0 };

	/* A part that already carries the fix must be left alone. */
	chip_reset();
	regs[DPS310_ERRATA_PROBE] = DPS310_ERRATA_OK_BIT;
	assert(dps310_probe(&baro, BUS, &c) == 0);
	assert(errata_writes == 0);
	dps310_remove(&baro);

	/* A part that does not gets the five-write sequence. */
	chip_reset();
	regs[DPS310_ERRATA_PROBE] = 0x00;
	assert(dps310_probe(&baro, BUS, &c) == 0);
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
	assert(dps310_probe(&baro, BUS, &c) == 0);
	assert(regs[DPS310_TMP_CFG] & DPS310_TMP_EXT);
	dps310_remove(&baro);

	chip_reset();
	regs[DPS310_COEF_SRCE] = 0x00;
	assert(dps310_probe(&baro, BUS, &c) == 0);
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
	assert(dps310_probe(&baro, BUS, &c) == 0);
	assert(regs[DPS310_CFG_REG] & DPS310_PRS_SHIFT_EN);
	assert(!(regs[DPS310_CFG_REG] & DPS310_TMP_SHIFT_EN));
	dps310_remove(&baro);

	/* Both low: neither shift bit. */
	chip_reset();
	c.pressure_osr = DPS310_OSR_8;
	c.temperature_osr = DPS310_OSR_1;
	assert(dps310_probe(&baro, BUS, &c) == 0);
	assert(regs[DPS310_CFG_REG] == 0);
	dps310_remove(&baro);

	/* Both high: both bits. At 1 Hz, so it fits the one-second budget. */
	chip_reset();
	c.pressure_rate = DPS310_RATE_1;
	c.temperature_rate = DPS310_RATE_1;
	c.pressure_osr = DPS310_OSR_128;
	c.temperature_osr = DPS310_OSR_64;
	assert(dps310_probe(&baro, BUS, &c) == 0);
	assert((regs[DPS310_CFG_REG] &
		(DPS310_PRS_SHIFT_EN | DPS310_TMP_SHIFT_EN)) ==
	       (DPS310_PRS_SHIFT_EN | DPS310_TMP_SHIFT_EN));
	dps310_remove(&baro);

	/* Against the datasheet, not the macro: P_SHIFT is bit 2, bit 4 is INT_PRS. */
	chip_reset();
	c = base_cfg();
	c.pressure_osr = DPS310_OSR_64;
	assert(dps310_probe(&baro, BUS, &c) == 0);
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
	stub_time_us = 777;
	assert(baro_read(&baro, &s) == 0);
	assert(s.t_us == 777);
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
	bus_error = ESP_ERR_TIMEOUT;
	s.t_us = 5;
	assert(baro_read(&baro, &s) == -ETIMEDOUT);
	assert(s.t_us == 5);
	bus_error = ESP_OK;

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
	allocs = 0;
	assert(dps310_probe(NULL, BUS, &c) == -EINVAL);
	assert(dps310_probe(&baro, BUS, NULL) == -EINVAL);
	assert(dps310_probe(&baro, NULL, &c) == -EINVAL);
	assert(added == 0 && allocs == 0 && "refused before calloc and the bus");
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
	int a = added, w = writes, r = reads, n = allocs;
	int32_t k[9];

	assert(dps310_probe(&baro, BUS, &c) == -EBUSY);
	assert(added == a && writes == w && reads == r && "no bus traffic");
	assert(allocs == n && "no allocation");
	assert(baro.priv == priv && dps310_coefficients(&baro, k) == 0 &&
	       "first binding intact");
	dps310_remove(&baro);

	/* Bound by another driver: refused too, and left exactly as it was. */
	assert(dps310_probe(&other, BUS, &c) == -EBUSY);
	assert(!strcmp(other.name, "other") && other.ops == &other_ops);

	/* A removed device can be probed afresh. */
	chip_reset();
	assert(dps310_probe(&baro, BUS, &c) == 0);
	dps310_remove(&baro);
}

int main(void)
{
	stub_tick_hz = 1000;

	test_probe();
	test_waits_for_ready();
	test_slow_tick();
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
