/*
 * bmp280 -- Bosch BMP280 barometer, as a baro chip driver.
 *
 * A piezo-resistive pressure die with a 20-bit ADC and twelve factory trim
 * words burnt into NVM. Cheap, everywhere, and good for about a metre of
 * *relative* altitude -- change -- against roughly 8 m absolute.
 *
 * I2C only, forced mode only: every baro_read() triggers one measurement,
 * waits it out and lets the chip fall back asleep. See the README for why.
 *
 * The BME280 is the same part with humidity added: same trim words, same
 * pressure and temperature compensation, so it would read correctly here. It
 * identifies itself as 0x60 and probe rejects it anyway, because this driver
 * only claims the BMP280.
 *
 * Samples come out of baro_read(). Everything about the chip -- registers,
 * bit fields, timings, the compensation -- is from memory of the BMP280
 * datasheet (src/bmp280/bmp280_regs.h). Nothing here has been run against a
 * physical chip.
 */
#ifndef BMP280_H
#define BMP280_H

#include <stdint.h>

#ifdef ESP_PLATFORM
#include "driver/i2c_master.h"
#else
#include CMSIS_device_header
#endif

#include "baro.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * The datasheet's recommended oversampling pairs, best and slowest first, so
 * that a zeroed config asks for the best. The times are the maximum forced
 * measurement per pair; a read does one.
 */
enum bmp280_osr {
	BMP280_OSR_ULTRA_HIGH	= 0,	/* p x16, t x2  -- 43.2 ms */
	BMP280_OSR_HIGH		= 1,	/* p x8,  t x1  -- 22.5 ms */
	BMP280_OSR_STANDARD	= 2,	/* p x4,  t x1  -- 13.3 ms */
	BMP280_OSR_LOW		= 3,	/* p x2,  t x1  --  8.7 ms */
	BMP280_OSR_ULTRA_LOW	= 4,	/* p x1,  t x1  --  6.4 ms */
};

/* NULL cfg = all defaults. */
struct bmp280_config {
#ifdef ESP_PLATFORM
	uint8_t address;		/* 0 selects 0x76 (SDO low); 0x77 when SDO high */
	uint32_t scl_hz;		/* 0 selects 400 kHz */
#else
	uint8_t address;		/* 7-bit; 0 selects 0x76, SDO low */
#endif
	enum bmp280_osr osr;		/* 0 selects ultra high resolution */
};

/*
 * bus is, on ESP-IDF, an i2c_master bus the app has already created; the
 * driver adds its device at cfg->scl_hz. On the STM32 HAL it is an I2C handle
 * the app has already set up with HAL_I2C_Init(), at 400 kHz or less; the bus
 * speed is the handle's, not this driver's.
 *
 * Check the chip ID, soft-reset, and read the 24 calibration bytes. The reset
 * leaves the chip asleep with the IIR filter off, which is what this driver
 * wants, so nothing else is configured until the first read. Fills
 * baro->name, ops and priv.
 *
 * baro_read() then triggers one forced measurement, waits the datasheet
 * maximum for the configured oversampling, burst-reads all six result bytes
 * and compensates them. On ESP-IDF it blocks the calling task: 50-60 ms at
 * the default oversampling and the default 100 Hz FreeRTOS tick, and never
 * less than 10-20 ms however fast the osr, because the wait is rounded up to
 * whole ticks; one task per baro_dev. On the STM32 HAL it busy-waits in
 * HAL_Delay(): 44-45 ms at the default oversampling and the HAL's default
 * 1 kHz tick, 7-8 ms at the fastest; one caller per baro_dev. It returns
 * -EIO if both raw values read as 0x80000, the reset value of the result
 * registers, which means no measurement ever completed; -ERANGE if the
 * pressure compensates to 0 or below.
 *
 * -EBUSY if baro is already bound (see struct baro_dev); nothing is touched.
 * -ENXIO if nothing ACKs at the address, decided before anything is allocated.
 * -ENODEV if the ID register is not 0x58 -- 0x60 is a BME280, which would read
 * pressure and temperature correctly but is not what this driver claims.
 * -EIO if dig_T1 or dig_P1 is zero, which is what a dead bus and an
 * unfinished NVM copy both look like, and what would divide by zero in the
 * pressure compensation.
 * -EINVAL for a NULL bus, an address other than 0x76/0x77, or an osr outside
 * the enum. Bus failures come back as the errno of the failed transfer.
 */
int bmp280_probe(struct baro_dev *baro,
#ifdef ESP_PLATFORM
		 i2c_master_bus_handle_t bus,
#else
		 I2C_HandleTypeDef *bus,
#endif
		 const struct bmp280_config *cfg);

/* Releases the device; the chip is already asleep between forced reads. Safe
 * on an unprobed baro_dev. */
void bmp280_remove(struct baro_dev *baro);

#ifdef __cplusplus
}
#endif

#endif /* BMP280_H */
