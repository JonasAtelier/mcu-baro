/*
 * bmp388 -- Bosch BMP388 barometer, as a baro chip driver.
 *
 * The BMP280's successor: 24-bit results, eleven pressure and three
 * temperature trim coefficients, and a floating-point compensation. Bosch
 * quote +/-8 Pa relative accuracy, about 0.66 m, and +/-50 Pa absolute.
 *
 * I2C only, forced mode only: every baro_read() triggers one measurement,
 * waits it out and lets the chip fall back asleep, as the BMP280 driver does.
 * See the README for why.
 *
 * The BMP390 identifies itself as 0x60 and is described as sharing this
 * register map; probe rejects it anyway, because this driver only claims the
 * BMP388.
 *
 * Samples come out of baro_read(). Registers, timings and the compensation
 * are from the BMP388 datasheet, BST-BMP388-DS001-07 rev 1.7 -- see
 * src/bmp388/bmp388_regs.h. Nothing here has been run against a physical chip.
 */
#ifndef BMP388_H
#define BMP388_H

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
 * The datasheet's oversampling presets (Table 6), best and slowest first, so
 * that a zeroed config asks for the best. The times are the maximum forced
 * measurement per preset (Table 22); a read does one.
 */
enum bmp388_osr {
	BMP388_OSR_HIGHEST	= 0,	/* p x32, t x2 -- 78.09 ms */
	BMP388_OSR_ULTRA_HIGH	= 1,	/* p x16, t x2 -- 41.89 ms */
	BMP388_OSR_HIGH		= 2,	/* p x8,  t x1 -- 21.53 ms */
	BMP388_OSR_STANDARD	= 3,	/* p x4,  t x1 -- 12.48 ms */
	BMP388_OSR_LOW		= 4,	/* p x2,  t x1 --  7.96 ms */
	BMP388_OSR_ULTRA_LOW	= 5,	/* p x1,  t x1 --  5.70 ms */
};

/* NULL cfg = all defaults. */
struct bmp388_config {
#ifdef ESP_PLATFORM
	uint8_t address;		/* 0 selects 0x76 (SDO low); 0x77 when SDO high */
	uint32_t scl_hz;		/* 0 selects 400 kHz */
#else
	uint8_t address;		/* 7-bit; 0 selects 0x76, SDO low */
#endif
	enum bmp388_osr osr;		/* 0 selects highest resolution */
};

/*
 * bus is, on ESP-IDF, an i2c_master bus the app has already created; the
 * driver adds its device at cfg->scl_hz. On the STM32 HAL it is an I2C handle
 * the app has already set up with HAL_I2C_Init(), at 400 kHz or less; the bus
 * speed is the handle's, not this driver's.
 *
 * Check the chip ID, wait for the command decoder, soft-reset, confirm the
 * reset finished, read the 21 trim bytes, and write the oversampling and an
 * output data rate slower than any measurement. The chip is left asleep with
 * the IIR filter off, as the reset leaves it. Fills baro->name, ops and priv.
 *
 * baro_read() then triggers one forced measurement, waits the datasheet
 * maximum for the configured oversampling, and burst-reads the error and
 * status registers with the six result bytes. On ESP-IDF it blocks the
 * calling task: 80-90 ms at the default oversampling and the default 100 Hz
 * FreeRTOS tick, and never less than 10-20 ms however fast the osr, because
 * the wait is rounded up to whole ticks; one task per baro_dev. On the STM32
 * HAL it busy-waits in HAL_Delay(): 79-80 ms at the default oversampling and
 * the HAL's default 1 kHz tick, 6-7 ms at the fastest; one caller per
 * baro_dev. It returns -EIO if ERR_REG
 * reports any error or either data-ready flag is clear, which means no new
 * measurement; -ERANGE if the pressure compensates to 0 or below, or to
 * something that is not a number.
 *
 * -EBUSY if baro is already bound (see struct baro_dev); nothing is touched.
 * -ENXIO if nothing ACKs at the address, decided before anything is allocated.
 * -ENODEV if the ID register is not 0x50 -- 0x60 is a BMP390, and the log
 * says so.
 * -ETIMEDOUT if STATUS.cmd_rdy stays clear for 10 ms before the reset.
 * -EIO if the soft reset is not reported complete, or NVM_PAR_T1 or
 * NVM_PAR_P5 reads zero, which is what a dead bus looks like.
 * -EINVAL for a NULL bus, an address other than 0x76/0x77, or an osr outside
 * the enum. Bus failures come back as the errno of the failed transfer.
 */
int bmp388_probe(struct baro_dev *baro,
#ifdef ESP_PLATFORM
		 i2c_master_bus_handle_t bus,
#else
		 I2C_HandleTypeDef *bus,
#endif
		 const struct bmp388_config *cfg);

/* Releases the device; the chip is already asleep between forced reads. Safe
 * on an unprobed baro_dev. */
void bmp388_remove(struct baro_dev *baro);

#ifdef __cplusplus
}
#endif

#endif /* BMP388_H */
