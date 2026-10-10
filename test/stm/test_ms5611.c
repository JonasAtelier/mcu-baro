/*
 * Host-side check of the driver against a fake MS5611 behind a fake HAL. No
 * hardware, no ARM toolchain -- see test/Makefile.
 *
 * The fake models what makes this chip awkward. It has no data-ready flag:
 * read the ADC before the conversion time is up and you get 0, and read the
 * PROM before the reload after reset is done and you get zeros. And it has
 * no register file: every transaction is a one-byte command, and an answer
 * is a separate read, so the fake serves Master_Transmit/Receive only.
 * HAL_Delay(n) waits n + 1 ticks, the first of which may be partial, so all
 * it guarantees is n whole milliseconds. That is all the fake credits.
 */
#include <assert.h>
#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "stm32_hal_stub.h"

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

static I2C_HandleTypeDef bus;
static uint16_t prom[8];
static uint16_t dev_addr;	/* 7-bit, of the last transfer */
static int allocs, frees, reads, resets, ncmds;
static uint8_t cmds[32];
static int answer = -1;		/* the read command awaiting its read; -1: none */
static uint32_t d1_value, d2_value;
static uint32_t pending;	/* result of the running conversion, 0 if none */
static long need_us, elapsed_us; /* since the last command that takes time */
static HAL_StatusTypeDef bus_status;	/* every transfer returns this if set */
static HAL_StatusTypeDef probe_status;	/* IsDeviceReady on a present chip */
static bool absent;		/* nothing ACKs at the address */
static uint16_t probed_addr;

static void chip_reset(void)
{
	absent = false;
	probed_addr = 0;
	memcpy(prom, DS_PROM, sizeof(prom));
	prom[7] = (uint16_t)((prom[7] & 0xFFF0) | ms5611_crc4(prom));
	d1_value = 9085466;	/* the datasheet example */
	d2_value = 8569150;
	pending = 0;
	need_us = elapsed_us = 0;
	allocs = frees = reads = resets = ncmds = 0;
	answer = -1;
	dev_addr = 0;
	bus_status = HAL_OK;
	probe_status = HAL_OK;
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

HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c,
					  uint16_t address, uint8_t *data,
					  uint16_t len, uint32_t timeout)
{
	uint8_t c = data[0], kind = c & 0xF0, step = c & 0x0F;

	assert(hi2c == &bus);
	assert(!(address & 1) && "the HAL takes the 7-bit address << 1");
	assert(len == 1 && "every MS5611 command is one byte");
	assert(timeout > 0);
	if (absent)
		return HAL_ERROR;
	if (bus_status != HAL_OK)
		return bus_status;
	dev_addr = address >> 1;
	if (ncmds < (int)sizeof(cmds))
		cmds[ncmds++] = c;
	answer = -1;

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
	} else if (c == MS5611_CMD_ADC_READ ||
		   (c >= MS5611_CMD_PROM_READ &&
		    c <= MS5611_CMD_PROM_READ + 14 && !(c & 1))) {
		answer = c;		/* the answer is the next read */
	} else {
		assert(0 && "unknown command");
	}
	return HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *hi2c,
					 uint16_t address, uint8_t *rx,
					 uint16_t rx_len, uint32_t timeout)
{
	bool done = elapsed_us >= need_us;
	int c = answer;

	assert(hi2c == &bus);
	assert(!(address & 1) && "the HAL takes the 7-bit address << 1");
	assert(timeout > 0);
	if (absent)
		return HAL_ERROR;
	if (bus_status != HAL_OK)
		return bus_status;
	assert(c >= 0 && "a read answers the read command just before it");
	answer = -1;
	reads++;
	dev_addr = address >> 1;

	if (c == MS5611_CMD_ADC_READ) {
		uint32_t v = done ? pending : 0;

		assert(rx_len == 3);
		rx[0] = (uint8_t)(v >> 16);
		rx[1] = (uint8_t)(v >> 8);
		rx[2] = (uint8_t)v;
		pending = 0;		/* a second read gives 0 as well */
		return HAL_OK;
	}

	assert(rx_len == 2);
	{
		uint16_t w = done ? prom[(c - MS5611_CMD_PROM_READ) / 2] : 0;

		rx[0] = (uint8_t)(w >> 8);
		rx[1] = (uint8_t)w;
	}
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

static struct baro_dev open_ok(const struct ms5611_config *cfg)
{
	struct baro_dev baro = { 0 };

	chip_reset();
	assert(ms5611_probe(&baro, &bus, cfg) == 0);
	assert(baro.ops && baro.priv);
	return baro;
}

/* ------------------------------------------------------------- tests ----- */

static void test_probe_defaults(void)
{
	struct baro_dev baro = open_ok(NULL);

	assert(!strcmp(baro.name, "ms5611"));
	assert(probed_addr == MS5611_ADDR_CSB_LOW && "NULL cfg selects 0x77");
	assert(dev_addr == MS5611_ADDR_CSB_LOW);
	assert(resets == 1 && cmds[0] == MS5611_CMD_RESET);
	ms5611_remove(&baro);
	assert(allocs == 1 && frees == 1);
	assert(!baro.ops && !baro.priv && "remove zeroes the binding");
}

static void test_config(void)
{
	struct ms5611_config c = { .address = MS5611_ADDR_CSB_HIGH,
				   .osr = MS5611_OSR_256 };
	struct baro_dev baro = open_ok(&c);
	struct baro_sample s;

	assert(probed_addr == MS5611_ADDR_CSB_HIGH);
	assert(dev_addr == MS5611_ADDR_CSB_HIGH);
	ncmds = 0;
	assert(baro_read(&baro, &s) == 0);
	assert(ncmds == 4 && cmds[0] == 0x50 && cmds[2] == 0x40 &&
	       "OSR 256, D2 first");
	assert(cmds[1] == MS5611_CMD_ADC_READ && cmds[3] == MS5611_CMD_ADC_READ);
	ms5611_remove(&baro);

	/* Refused before anything touches the bus, and nothing allocated. */
	chip_reset();
	c.address = 0x60;
	assert(ms5611_probe(&baro, &bus, &c) == -EINVAL);
	c.address = 0;
	c.osr = (enum ms5611_osr)5;
	assert(ms5611_probe(&baro, &bus, &c) == -EINVAL);
	assert(ncmds == 0 && reads == 0 && probed_addr == 0 && allocs == 0);
	assert(!baro.ops && !baro.priv);
}

static void test_read_datasheet_example(void)
{
	struct baro_dev baro = open_ok(NULL);
	struct baro_sample s;

	ncmds = 0;
	stub_tick = 31337;
	assert(baro_read(&baro, &s) == 0);
	assert(ncmds == 4 && cmds[0] == 0x58 && cmds[2] == 0x48 &&
	       "OSR 4096, D2 first");
	assert(s.pressure_pa == 100009.0f);
	assert(fabsf(s.temp_c - 20.07f) < 1e-4f);
	assert(s.t_us == 31337000);
	ms5611_remove(&baro);
}

static void test_waits_long_enough_at_every_osr(void)
{
	/*
	 * HAL_Delay() only takes whole milliseconds, so OSR 256's 0.60 ms
	 * becomes 1 ms. Each must still cover the unrounded datasheet maximum.
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
			assert(ms5611_probe(&baro, &bus, NULL) == -ENODEV);
			assert(allocs == frees && "nothing leaked");
			assert(!baro.ops && !baro.priv);
		}
	}
	assert(strstr(stub_log_last, "baro: ms5611: PROM at 0x77 fails"));

	/*
	 * Those four bits really are unchecked. Pinned, so a change to the CRC
	 * mask shows up here rather than on a real part.
	 */
	chip_reset();
	prom[7] ^= 0x00F0;
	assert(ms5611_probe(&baro, &bus, NULL) == 0);
	ms5611_remove(&baro);

	/* An all-zero PROM has CRC 0 and passes the CRC on its own. */
	chip_reset();
	memset(prom, 0, sizeof(prom));
	assert(ms5611_probe(&baro, &bus, NULL) == -ENODEV);
	assert(allocs == 1 && frees == 1);
}

static void test_no_answer(void)
{
	struct baro_dev baro = { 0 };
	struct baro_sample s;

	/*
	 * Nothing ACKs: -ENXIO, decided by HAL_I2C_IsDeviceReady() before
	 * anything is allocated or sent.
	 */
	chip_reset();
	absent = true;
	assert(ms5611_probe(&baro, &bus, NULL) == -ENXIO);
	assert(probed_addr == MS5611_ADDR_CSB_LOW);
	assert(allocs == 0 && ncmds == 0 && reads == 0);
	assert(!baro.ops && !baro.priv);

	/* A bus that is busy or hangs during the presence check is not absence. */
	chip_reset();
	probe_status = HAL_BUSY;	/* also: a handle never initialised */
	assert(ms5611_probe(&baro, &bus, NULL) == -EAGAIN);
	assert(allocs == 0 && ncmds == 0 && reads == 0);
	chip_reset();
	probe_status = HAL_TIMEOUT;
	assert(ms5611_probe(&baro, &bus, NULL) == -ETIMEDOUT);
	assert(allocs == 0);

	/* A NACK in the middle of a transfer is an I/O error, not absence. */
	chip_reset();
	bus_status = HAL_ERROR;
	assert(ms5611_probe(&baro, &bus, NULL) == -EIO);
	assert(allocs == 1 && frees == 1);
	chip_reset();
	/* Not -EBUSY: from a probe that means "already bound". */
	bus_status = HAL_BUSY;
	assert(ms5611_probe(&baro, &bus, NULL) == -EAGAIN);
	assert(allocs == 1 && frees == 1);
	assert(!baro.ops && !baro.priv);

	/* A bus that dies after probe fails the read. */
	baro = open_ok(NULL);
	bus_status = HAL_TIMEOUT;
	assert(baro_read(&baro, &s) == -ETIMEDOUT);
	bus_status = HAL_OK;
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
	assert(ms5611_probe(NULL, &bus, NULL) == -EINVAL);
	assert(ms5611_probe(&baro, NULL, NULL) == -EINVAL);
	assert(probed_addr == 0 && allocs == 0 && ncmds == 0);
	ms5611_remove(NULL);
	ms5611_remove(&baro);			/* unprobed: a no-op */

	/* Another chip's baro_dev is not ours to remove. */
	ms5611_remove(&other);
	assert(other.ops == &other_ops && other.priv == prom);
	assert(frees == 0);
}

static void test_second_probe_is_refused(void)
{
	static const struct baro_ops other_ops = { .read = other_read };
	struct baro_dev other = { .name = "other", .ops = &other_ops };
	struct baro_dev baro = open_ok(NULL);
	void *priv = baro.priv;
	int c = ncmds, r = reads, n = allocs;
	struct baro_sample s;

	probed_addr = 0;
	assert(ms5611_probe(&baro, &bus, NULL) == -EBUSY);
	assert(probed_addr == 0 && ncmds == c && reads == r &&
	       "no bus traffic");
	assert(allocs == n && "no allocation");
	assert(baro.priv == priv && baro_read(&baro, &s) == 0 &&
	       "first binding intact");
	ms5611_remove(&baro);

	/* Bound by another driver: refused too, and left exactly as it was. */
	assert(ms5611_probe(&other, &bus, NULL) == -EBUSY);
	assert(!strcmp(other.name, "other") && other.ops == &other_ops);

	/* A removed device can be probed afresh. */
	chip_reset();
	assert(ms5611_probe(&baro, &bus, NULL) == 0);
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
