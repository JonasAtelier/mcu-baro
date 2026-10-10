/*
 * Host-side check of the driver against a fake MS5611. No hardware, no ESP
 * toolchain -- see test/Makefile.
 *
 * The fake models what makes this chip awkward. It has no data-ready flag:
 * read the ADC before the conversion time is up and you get 0, and read the
 * PROM before the reload after reset is done and you get zeros. And FreeRTOS
 * delays are coarse: vTaskDelay(n) is only guaranteed n - 1 whole ticks, so
 * that is all the fake credits. The stub tick is the ESP-IDF default, 100 Hz.
 * It has no register file either: every transaction is a one-byte command,
 * and an answer is a separate read, so the fake serves i2c_master_transmit()
 * and i2c_master_receive() only; a repeated start would reach the weak
 * i2c_master_transmit_receive() in idf_stubs.c and abort.
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
#include "ms5611.h"
#include "ms5611_math.h"
#include "ms5611_proto.h"

/* The datasheet's typical calibration, C1..C6 in words 1..6. */
static const uint16_t DS_PROM[8] = { 0x0000, 40127, 36924, 23317, 23282,
				     33464, 28312, 0x1230 };

/*
 * Datasheet maximum conversion time per OSR, 4096 first, in microseconds --
 * the unrounded figures, deliberately not the driver's table.
 */
static const long CONV_US[5] = { 9040, 4540, 2280, 1170, 600 };
#define RESET_US	2800

static uint16_t prom[8];
static uint16_t dev_addr;
static int attached, added, reads, resets, ncmds;
static uint8_t cmds[32];
static uint32_t d1_value, d2_value;
static uint32_t pending;	/* result of the running conversion, 0 if none */
static int answer = -1;		/* the read command the next read answers */
static long need_us, elapsed_us; /* since the last command that takes time */
static esp_err_t bus_error;	/* every transfer fails with this if set */
static bool absent;		/* nothing ACKs at the address */
static uint16_t probed_addr;

#define BUS ((i2c_master_bus_handle_t)(void *)prom)

static void chip_reset(void)
{
	absent = false;
	probed_addr = 0;
	memcpy(prom, DS_PROM, sizeof(prom));
	prom[7] = (uint16_t)((prom[7] & 0xFFF0) | ms5611_crc4(prom));
	d1_value = 9085466;	/* the datasheet example */
	d2_value = 8569150;
	pending = 0;
	answer = -1;
	need_us = elapsed_us = 0;
	added = reads = resets = ncmds = 0;
	bus_error = ESP_OK;
}

esp_err_t i2c_master_bus_add_device(i2c_master_bus_handle_t bus,
				    const i2c_device_config_t *cfg,
				    i2c_master_dev_handle_t *out)
{
	(void)bus;
	assert(cfg->dev_addr_length == I2C_ADDR_BIT_LEN_7);
	dev_addr = cfg->device_address;
	attached++;
	added++;
	*out = (i2c_master_dev_handle_t)(void *)prom;
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
	uint8_t c = data[0], kind = c & 0xF0, step = c & 0x0F;

	(void)d;
	(void)t;
	assert(len == 1 && "every MS5611 command is one byte");
	if (absent)
		return ESP_ERR_INVALID_STATE;
	if (bus_error != ESP_OK)
		return bus_error;
	answer = -1;
	if (c == MS5611_CMD_ADC_READ ||
	    (c >= MS5611_CMD_PROM_READ && c <= MS5611_CMD_PROM_READ + 14 &&
	     !(c & 1))) {
		answer = c;	/* the answer is the next read, counted there */
		return ESP_OK;
	}
	if (ncmds < (int)sizeof(cmds))
		cmds[ncmds++] = c;

	if (c == MS5611_CMD_RESET) {
		resets++;
		pending = 0;
		need_us = RESET_US;
		elapsed_us = 0;
	} else if (kind == MS5611_CMD_CONV_D1 || kind == MS5611_CMD_CONV_D2) {
		assert(step <= 8 && !(step & 1));
		pending = kind == MS5611_CMD_CONV_D1 ? d1_value : d2_value;
		need_us = CONV_US[(8 - step) / 2];
		elapsed_us = 0;
	} else {
		assert(0 && "unknown command");
	}
	return ESP_OK;
}

esp_err_t i2c_master_receive(i2c_master_dev_handle_t d, uint8_t *rx,
			     size_t rx_len, int t)
{
	bool done = elapsed_us >= need_us;
	int c = answer;

	(void)d;
	(void)t;
	if (absent)
		return ESP_ERR_INVALID_STATE;
	if (bus_error != ESP_OK)
		return bus_error;
	assert(c >= 0 && "a read answers the read command just before it");
	answer = -1;
	reads++;

	if (c == MS5611_CMD_ADC_READ) {
		uint32_t v = done ? pending : 0;

		assert(rx_len == 3);
		rx[0] = (uint8_t)(v >> 16);
		rx[1] = (uint8_t)(v >> 8);
		rx[2] = (uint8_t)v;
		pending = 0;		/* a second read gives 0 as well */
		return ESP_OK;
	}

	assert(rx_len == 2);
	{
		uint16_t w = done ? prom[(c - MS5611_CMD_PROM_READ) / 2] : 0;

		rx[0] = (uint8_t)(w >> 8);
		rx[1] = (uint8_t)w;
	}
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

static struct baro_dev open_ok(const struct ms5611_config *cfg)
{
	struct baro_dev baro = { 0 };

	chip_reset();
	assert(ms5611_probe(&baro, BUS, cfg) == 0);
	assert(baro.ops && baro.priv);
	return baro;
}

/* ------------------------------------------------------------- tests ----- */

static void test_probe_defaults(void)
{
	struct baro_dev baro = open_ok(NULL);

	assert(!strcmp(baro.name, "ms5611"));
	assert(dev_addr == MS5611_ADDR_CSB_LOW && "NULL cfg selects 0x77");
	assert(resets == 1 && cmds[0] == MS5611_CMD_RESET);
	ms5611_remove(&baro);
	assert(attached == 0);
	assert(!baro.ops && !baro.priv && "remove zeroes the binding");
}

static void test_config(void)
{
	struct ms5611_config c = { .address = MS5611_ADDR_CSB_HIGH,
				   .osr = MS5611_OSR_256 };
	struct baro_dev baro = open_ok(&c);
	struct baro_sample s;

	assert(dev_addr == MS5611_ADDR_CSB_HIGH);
	ncmds = 0;
	assert(baro_read(&baro, &s) == 0);
	assert(ncmds == 2 && cmds[0] == 0x50 && cmds[1] == 0x40 &&
	       "OSR 256, D2 first");
	ms5611_remove(&baro);

	/* Refused before anything touches the bus, and nothing allocated. */
	chip_reset();
	allocs = 0;
	c.address = 0x60;
	assert(ms5611_probe(&baro, BUS, &c) == -EINVAL);
	c.address = 0;
	c.osr = (enum ms5611_osr)5;
	assert(ms5611_probe(&baro, BUS, &c) == -EINVAL);
	assert(ncmds == 0 && attached == 0 && added == 0 && allocs == 0);
	assert(!baro.ops && !baro.priv);
}

static void test_read_datasheet_example(void)
{
	struct baro_dev baro = open_ok(NULL);
	struct baro_sample s;

	ncmds = 0;
	stub_time_us = 31337;
	assert(baro_read(&baro, &s) == 0);
	assert(ncmds == 2 && cmds[0] == 0x58 && cmds[1] == 0x48 &&
	       "OSR 4096, D2 first");
	assert(s.pressure_pa == 100009.0f);
	assert(fabsf(s.temp_c - 20.07f) < 1e-4f);
	assert(s.t_us == 31337);
	ms5611_remove(&baro);
}

static void test_waits_long_enough_at_every_osr(void)
{
	/*
	 * At 100 Hz pdMS_TO_TICKS turns every one of these waits into 0 or 1
	 * tick. Each must still cover the datasheet maximum.
	 */
	for (int osr = MS5611_OSR_4096; osr <= MS5611_OSR_256; osr++) {
		struct ms5611_config c = { .osr = (enum ms5611_osr)osr };
		struct baro_dev baro = open_ok(&c);
		struct baro_sample s;

		assert(baro_read(&baro, &s) == 0);
		ms5611_remove(&baro);
	}
}

static void test_adc_zero_is_an_error(void)
{
	struct baro_dev baro = open_ok(NULL);
	struct baro_sample s = { .pressure_pa = 1.0f, .temp_c = 1.0f,
				 .t_us = 3 };

	/*
	 * 0 is the chip's answer to a read with no finished conversion. Pushed
	 * through the compensation it becomes a nonsense number, not an error.
	 */
	d2_value = 0;
	assert(baro_read(&baro, &s) == -EIO);
	d2_value = 8569150;
	d1_value = 0;
	assert(baro_read(&baro, &s) == -EIO);
	assert(s.pressure_pa == 1.0f && s.temp_c == 1.0f && s.t_us == 3 &&
	       "sample untouched");
	ms5611_remove(&baro);
}

static void test_bad_prom(void)
{
	struct baro_dev baro = { 0 };

	/*
	 * Any single flipped bit fails the CRC -- except bits 4..7 of word 7,
	 * which AN520 leaves out of the calculation along with the CRC nibble.
	 */
	for (int w = 0; w < 8; w++) {
		for (int b = 0; b < 16; b++) {
			if (w == 7 && b >= 4 && b <= 7)
				continue;
			chip_reset();
			prom[w] ^= (uint16_t)(1u << b);
			assert(ms5611_probe(&baro, BUS, NULL) == -ENODEV);
			assert(attached == 0 && "device detached on failure");
			assert(!baro.ops && !baro.priv);
		}
	}
	assert(strstr(stub_log_last, "E baro: ms5611: PROM at 0x77 fails"));

	/*
	 * Those four bits really are unchecked. Pinned, so a change to the CRC
	 * mask shows up here rather than on a real part.
	 */
	chip_reset();
	prom[7] ^= 0x00F0;
	assert(ms5611_probe(&baro, BUS, NULL) == 0);
	ms5611_remove(&baro);

	/* An all-zero PROM has CRC 0 and passes the CRC on its own. */
	chip_reset();
	memset(prom, 0, sizeof(prom));
	assert(ms5611_probe(&baro, BUS, NULL) == -ENODEV);
	assert(attached == 0);
}

static void test_no_answer(void)
{
	struct baro_dev baro = { 0 };
	struct baro_sample s;

	/*
	 * Nothing ACKs: -ENXIO, decided by i2c_master_probe() before anything
	 * is allocated or added. A NACK in the middle of a transfer (IDF 5.5:
	 * ESP_ERR_INVALID_STATE) is an I/O error, not absence.
	 */
	chip_reset();
	absent = true;
	allocs = 0;
	assert(ms5611_probe(&baro, BUS, NULL) == -ENXIO);
	assert(probed_addr == MS5611_ADDR_CSB_LOW);
	assert(allocs == 0 && added == 0 && attached == 0);
	assert(!baro.ops && !baro.priv);
	chip_reset();
	bus_error = ESP_ERR_INVALID_STATE;
	assert(ms5611_probe(&baro, BUS, NULL) == -EIO);
	assert(attached == 0);
	chip_reset();
	bus_error = ESP_FAIL;
	assert(ms5611_probe(&baro, BUS, NULL) == -EIO);
	assert(attached == 0);
	assert(!baro.ops && !baro.priv);

	/* A bus that dies after probe fails the read. */
	baro = open_ok(NULL);
	bus_error = ESP_ERR_TIMEOUT;
	assert(baro_read(&baro, &s) == -ETIMEDOUT);
	bus_error = ESP_OK;
	ms5611_remove(&baro);
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
				  .priv = prom };
	struct baro_dev baro = { 0 };

	chip_reset();
	assert(ms5611_probe(NULL, BUS, NULL) == -EINVAL);
	assert(ms5611_probe(&baro, NULL, NULL) == -EINVAL);
	assert(added == 0 && ncmds == 0);
	ms5611_remove(NULL);
	ms5611_remove(&baro);			/* unprobed: a no-op */

	/* Another chip's baro_dev is not ours to remove. */
	ms5611_remove(&other);
	assert(other.ops == &other_ops && other.priv == prom);
	assert(attached == 0);
}

static void test_second_probe_is_refused(void)
{
	static const struct baro_ops other_ops = { .read = other_read };
	struct baro_dev other = { .name = "other", .ops = &other_ops };
	struct baro_dev baro = open_ok(NULL);
	void *priv = baro.priv;
	int a = added, c = ncmds, r = reads, n = allocs;
	struct baro_sample s;

	assert(ms5611_probe(&baro, BUS, NULL) == -EBUSY);
	assert(added == a && ncmds == c && reads == r && "no bus traffic");
	assert(allocs == n && "no allocation");
	assert(baro.priv == priv && baro_read(&baro, &s) == 0 &&
	       "first binding intact");
	ms5611_remove(&baro);

	/* Bound by another driver: refused too, and left exactly as it was. */
	assert(ms5611_probe(&other, BUS, NULL) == -EBUSY);
	assert(!strcmp(other.name, "other") && other.ops == &other_ops);

	/* A removed device can be probed afresh. */
	chip_reset();
	assert(ms5611_probe(&baro, BUS, NULL) == 0);
	ms5611_remove(&baro);
}

int main(void)
{
	test_probe_defaults();
	test_config();
	test_read_datasheet_example();
	test_waits_long_enough_at_every_osr();
	test_adc_zero_is_an_error();
	test_bad_prom();
	test_no_answer();
	test_null_and_foreign_arguments();
	test_second_probe_is_refused();
	printf("all ms5611 driver checks passed\n");
	return 0;
}
