/*
 * Host-side check of the driver against a fake BMP280. No hardware, no ESP
 * toolchain -- see test/Makefile.
 *
 * The fake is a 256-byte register file plus the two things on this chip that
 * take time: the start-up after a soft reset, during which the trim registers
 * read zeros, and a forced measurement, during which the result registers
 * hold whatever they held before -- 0x80000 on a part that has just been
 * reset. It charges the datasheet *maximum* times, computed from the formula
 * rather than from the driver's rounded table, so the driver cannot pass by
 * agreeing with itself.
 *
 * FreeRTOS delays are coarse: vTaskDelay(n) is only guaranteed n - 1 whole
 * ticks, so that is all the fake credits. The stub tick is the ESP-IDF
 * default, 100 Hz, where pdMS_TO_TICKS(9) is 0.
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

static uint8_t regs[256];
static uint8_t chip_id;
static uint16_t cal[12];
static int32_t raw_t, raw_p;	/* what the next finished measurement yields */
static long need_us, elapsed_us;
static enum { P_NONE, P_RESET, P_MEAS } pending;
static uint16_t dev_addr;
static uint32_t dev_hz;
static int attached, added, writes, reads, resets, bursts;
static uint8_t last_ctrl, last_config;
static esp_err_t bus_error;	/* every transfer fails with this if set */
static bool absent;		/* nothing ACKs at the address */
static uint16_t probed_addr;

#define BUS ((i2c_master_bus_handle_t)(void *)regs)

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
	memset(regs, 0, sizeof(regs));
	chip_id = BMP280_CHIP_ID;
	memcpy(cal, CAL_WORDS, sizeof(cal));
	raw_t = RAW_T;
	raw_p = RAW_P;
	added = writes = reads = resets = bursts = 0;
	last_ctrl = last_config = 0xFF;
	bus_error = ESP_OK;
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

esp_err_t i2c_master_bus_add_device(i2c_master_bus_handle_t bus,
				    const i2c_device_config_t *cfg,
				    i2c_master_dev_handle_t *out)
{
	(void)bus;
	assert(cfg->dev_addr_length == I2C_ADDR_BIT_LEN_7);
	dev_addr = cfg->device_address;
	dev_hz = cfg->scl_speed_hz;
	attached++;
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
	attached--;
	return ESP_OK;
}

esp_err_t i2c_master_transmit(i2c_master_dev_handle_t d, const uint8_t *data,
			      size_t len, int t)
{
	uint8_t reg, val;

	(void)d;
	(void)t;
	assert(len == 2 && "a write is the register address then one byte");
	if (absent)
		return ESP_ERR_INVALID_STATE;
	if (bus_error != ESP_OK)
		return bus_error;
	reg = data[0];
	val = data[1];
	writes++;

	if (reg == BMP280_REG_RESET) {
		if (val == BMP280_RESET_WORD)
			chip_soft_reset();	/* any other value is ignored */
		return ESP_OK;
	}
	if (reg == BMP280_REG_CONFIG) {
		last_config = val;
		regs[reg] = val;
		return ESP_OK;
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
	return ESP_OK;
}

esp_err_t i2c_master_transmit_receive(i2c_master_dev_handle_t d,
				      const uint8_t *tx, size_t tx_len,
				      uint8_t *rx, size_t rx_len, int t)
{
	(void)d;
	(void)t;
	assert(tx_len == 1 && "a read is one address byte, then the bytes back");
	if (absent)
		return ESP_ERR_INVALID_STATE;
	if (bus_error != ESP_OK)
		return bus_error;
	reads++;
	settle();

	if (tx[0] == BMP280_REG_ID) {
		assert(rx_len == 1);
		rx[0] = chip_id;
		return ESP_OK;
	}
	if (tx[0] == BMP280_REG_PRESS_MSB) {
		assert(rx_len == BMP280_RESULT_LEN && "both results in one burst");
		bursts++;
	}
	assert((size_t)tx[0] + rx_len <= sizeof(regs));
	memcpy(rx, &regs[tx[0]], rx_len);	/* reads auto-increment */
	return ESP_OK;
}

/* Credit only the whole ticks vTaskDelay is guaranteed to wait. */
void vTaskDelay(TickType_t t)
{
	if (t > 0)
		elapsed_us += (long)(t - 1) * portTICK_PERIOD_MS * 1000;
}

TickType_t xTaskGetTickCount(void)
{
	return 0;
}

/* Counts the drivers' calloc() calls; linked with -Wl,--wrap=calloc. */
static int allocs;
void *__real_calloc(size_t n, size_t size);
void *__wrap_calloc(size_t n, size_t size)
{
	allocs++;
	return __real_calloc(n, size);
}

static struct baro_dev open_ok(const struct bmp280_config *cfg)
{
	struct baro_dev baro = { 0 };

	chip_power_on();
	assert(bmp280_probe(&baro, BUS, cfg) == 0);
	assert(baro.ops && baro.priv);
	return baro;
}

/* ------------------------------------------------------------- tests ----- */

static void test_probe_defaults(void)
{
	struct baro_dev baro = open_ok(NULL);

	assert(!strcmp(baro.name, "bmp280"));
	assert(dev_addr == BMP280_ADDR_SDO_LOW && "NULL cfg selects 0x76");
	assert(dev_hz == 400000);
	assert(resets == 1 && "probe soft-resets once");
	assert(last_config == 0xFF &&
	       "config untouched: the reset already zeroed it");
	assert(regs[BMP280_REG_CONFIG] == 0 && "IIR off, spi3w off");
	assert(regs[BMP280_REG_CTRL_MEAS] == 0 && "the chip is left asleep");
	bmp280_remove(&baro);
	assert(attached == 0);
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
	assert(bmp280_probe(&baro, BUS, NULL) == -ENODEV);
	assert(strstr(stub_log_last, "E baro: bmp280: "));
	assert(strstr(stub_log_last, "that is a BME280"));
	assert(resets == 0 && "rejected before anything is written");
	assert(attached == 0 && "device detached again on failure");
	assert(!baro.ops && !baro.priv && "a failed probe leaves it unbound");

	/* Anything else, including the 0x00 or 0xFF of a floating bus. */
	chip_power_on();
	chip_id = 0x00;
	assert(bmp280_probe(&baro, BUS, NULL) == -ENODEV);
	chip_power_on();
	chip_id = 0xFF;
	assert(bmp280_probe(&baro, BUS, NULL) == -ENODEV);
	assert(attached == 0);
}

static void test_config(void)
{
	struct bmp280_config c = { .address = BMP280_ADDR_SDO_HIGH,
				   .scl_hz = 100000,
				   .osr = BMP280_OSR_ULTRA_LOW };
	struct baro_dev baro = open_ok(&c);
	struct baro_sample s;

	assert(dev_addr == BMP280_ADDR_SDO_HIGH && dev_hz == 100000);
	assert(baro_read(&baro, &s) == 0);
	assert(last_ctrl ==
	       bmp280_ctrl_meas(BMP280_OSR_ULTRA_LOW, BMP280_MODE_FORCED));
	assert(last_ctrl == 0x25 && "osrs_t x1, osrs_p x1, forced");
	bmp280_remove(&baro);

	/* Refused before anything touches the bus, and nothing allocated. */
	chip_power_on();
	allocs = 0;
	c.address = 0x60;
	assert(bmp280_probe(&baro, BUS, &c) == -EINVAL);
	c.address = 0;
	c.osr = (enum bmp280_osr)5;
	assert(bmp280_probe(&baro, BUS, &c) == -EINVAL);
	assert(writes == 0 && added == 0 && allocs == 0);
	assert(!baro.ops && !baro.priv);
}

static void test_read(void)
{
	struct baro_dev baro = open_ok(NULL);
	struct baro_sample s;

	writes = bursts = 0;
	stub_time_us = 5555;
	assert(baro_read(&baro, &s) == 0);
	assert(writes == 1 && bursts == 1 && "one trigger, one burst");
	assert(last_ctrl == 0x55 && "ultra high resolution, forced");
	assert(fabsf(s.temp_c - EXPECT_C) < 0.005f);
	assert(fabsf(s.pressure_pa - EXPECT_PA) < 0.01f);
	assert(s.t_us == 5555);

	/* A second read triggers again: forced mode measures once and sleeps. */
	assert(baro_read(&baro, &s) == 0);
	assert(writes == 2 && bursts == 2);
	bmp280_remove(&baro);
}

static void test_waits_long_enough_at_every_osr(void)
{
	/*
	 * At 100 Hz pdMS_TO_TICKS turns these waits into 0 to 4 ticks. Each
	 * must still cover the datasheet maximum, or the fake serves the stale
	 * 0x80000 the reset left behind and the read fails.
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
	assert(bmp280_probe(&baro, BUS, NULL) == -EIO);
	assert(attached == 0);
	assert(!baro.ops && !baro.priv);

	/* dig_P1 = 0 alone: the divide-by-zero in the compensation. */
	chip_power_on();
	cal[3] = 0;
	assert(bmp280_probe(&baro, BUS, NULL) == -EIO);
	assert(attached == 0);

	/* dig_T1 = 0 alone. */
	chip_power_on();
	cal[0] = 0;
	assert(bmp280_probe(&baro, BUS, NULL) == -EIO);
	assert(attached == 0);

	/* A signed word reading zero is legal and must not be rejected. */
	chip_power_on();
	cal[1] = 0;
	assert(bmp280_probe(&baro, BUS, NULL) == 0);
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
	 * Nothing ACKs: -ENXIO, decided by i2c_master_probe() before anything
	 * is allocated or added. A NACK in the middle of a transfer (IDF 5.5:
	 * ESP_ERR_INVALID_STATE) is an I/O error, not absence.
	 */
	chip_power_on();
	absent = true;
	allocs = 0;
	assert(bmp280_probe(&baro, BUS, NULL) == -ENXIO);
	assert(probed_addr == BMP280_ADDR_SDO_LOW);
	assert(allocs == 0 && added == 0 && attached == 0);
	assert(!baro.ops && !baro.priv);
	chip_power_on();
	bus_error = ESP_ERR_INVALID_STATE;
	assert(bmp280_probe(&baro, BUS, NULL) == -EIO);
	assert(attached == 0);
	chip_power_on();
	bus_error = ESP_FAIL;
	assert(bmp280_probe(&baro, BUS, NULL) == -EIO);
	assert(attached == 0);
	assert(!baro.ops && !baro.priv);

	/*
	 * A bus that dies after probe: the read reports it rather than the
	 * stale register contents.
	 */
	baro = open_ok(NULL);
	bus_error = ESP_ERR_TIMEOUT;
	assert(baro_read(&baro, &s) == -ETIMEDOUT);
	bus_error = ESP_OK;
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
	assert(bmp280_probe(NULL, BUS, NULL) == -EINVAL);
	assert(bmp280_probe(&baro, NULL, NULL) == -EINVAL);
	assert(attached == 0 && writes == 0);
	bmp280_remove(NULL);
	bmp280_remove(&baro);			/* unprobed: a no-op */

	/* Another chip's baro_dev is not ours to remove. */
	bmp280_remove(&other);
	assert(other.ops == &other_ops && other.priv == regs);
}

static void test_second_probe_is_refused(void)
{
	static const struct baro_ops other_ops = { .read = other_read };
	struct baro_dev other = { .name = "other", .ops = &other_ops };
	struct baro_dev baro = open_ok(NULL);
	void *priv = baro.priv;
	int a = added, w = writes, r = reads, n = allocs;
	struct baro_sample s;

	assert(bmp280_probe(&baro, BUS, NULL) == -EBUSY);
	assert(added == a && writes == w && reads == r && "no bus traffic");
	assert(allocs == n && "no allocation");
	assert(baro.priv == priv && baro_read(&baro, &s) == 0 &&
	       "first binding intact");
	bmp280_remove(&baro);

	/* Bound by another driver: refused too, and left exactly as it was. */
	assert(bmp280_probe(&other, BUS, NULL) == -EBUSY);
	assert(!strcmp(other.name, "other") && other.ops == &other_ops);

	/* A removed device can be probed afresh. */
	chip_power_on();
	assert(bmp280_probe(&baro, BUS, NULL) == 0);
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
