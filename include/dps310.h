/*
 * dps310 -- Infineon DPS310 barometric pressure sensor, as a baro chip
 * driver.
 *
 * Pressure to about +/-0.002 hPa of relative precision, which is roughly two
 * centimetres of altitude. Temperature comes with it, because the pressure
 * compensation needs it anyway. The part measures in the background; the
 * driver reads the latest result.
 *
 * I2C only. The part also does SPI; that is not implemented here.
 *
 * Samples come out of baro_read(). Everything below that is DPS310 specific
 * takes the struct baro_dev that dps310_probe() filled, and returns -EINVAL
 * if handed a device some other driver probed.
 *
 * Register provenance is in src/dps310/dps310_regs.h. Nothing here has been run
 * against a physical chip.
 */
#ifndef DPS310_H
#define DPS310_H

#include <stdbool.h>
#include <stddef.h>
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
 * Oversampling. Above 8x the driver sets the shift bit for you. 128x takes
 * about 207 ms per measurement, so it does not combine with a fast rate.
 */
enum dps310_osr {
	DPS310_OSR_1	= 0,
	DPS310_OSR_2	= 1,
	DPS310_OSR_4	= 2,
	DPS310_OSR_8	= 3,
	DPS310_OSR_16	= 4,
	DPS310_OSR_32	= 5,
	DPS310_OSR_64	= 6,
	DPS310_OSR_128	= 7,
};

/* Measurements per second in background mode: 1, 2, 4 ... 128. */
enum dps310_rate {
	DPS310_RATE_1	= 0,
	DPS310_RATE_2	= 1,
	DPS310_RATE_4	= 2,
	DPS310_RATE_8	= 3,
	DPS310_RATE_16	= 4,
	DPS310_RATE_32	= 5,
	DPS310_RATE_64	= 6,
	DPS310_RATE_128	= 7,
};

struct dps310_config {
#ifdef ESP_PLATFORM
	uint8_t address;		/* 0 selects 0x77, the usual breakout default */
	uint32_t scl_hz;		/* 0 selects 400 kHz */
#else
	uint8_t address;		/* 7-bit; 0 selects 0x77, SDO high */
#endif

	enum dps310_rate pressure_rate;
	enum dps310_osr pressure_osr;
	enum dps310_rate temperature_rate;
	enum dps310_osr temperature_osr;
};

/*
 * bus is, on ESP-IDF, an i2c_master bus the app has already created; the
 * driver adds its device at cfg->scl_hz. On the STM32 HAL it is an I2C handle
 * the app has already set up with HAL_I2C_Init(), at 400 kHz or less; the bus
 * speed is the handle's, not this driver's.
 *
 * Reset, wait for the sensor and its calibration to come up, apply the
 * erratum fix if this part needs it, read the nine factory coefficients, and
 * start measuring in the background. Fills baro->name, ops and priv. cfg is
 * required.
 *
 * -EBUSY if baro is already bound (see struct baro_dev); nothing is touched.
 * -ENXIO if nothing ACKs at the address, decided before anything is allocated.
 * -ENODEV if the product ID is not 0x10.
 * -ETIMEDOUT if the sensor or its coefficients never report ready.
 * -EINVAL for a NULL bus or cfg, an address other than 0x76/0x77, a rate or
 * oversampling outside the enums, or a combination that cannot finish its
 * measurements inside a second (rate x measurement time, pressure plus
 * temperature, must stay under 990 ms).
 * Bus failures come back as the errno of the failed transfer.
 */
int dps310_probe(struct baro_dev *baro,
#ifdef ESP_PLATFORM
		 i2c_master_bus_handle_t bus,
#else
		 I2C_HandleTypeDef *bus,
#endif
		 const struct dps310_config *cfg);

/* Puts the chip idle and releases it. Safe on an unprobed baro_dev. */
void dps310_remove(struct baro_dev *baro);

/*
 * Raw 24-bit results, before compensation. Mostly useful for debugging a
 * reading you do not believe.
 */
int dps310_read_raw(struct baro_dev *baro, int32_t *pressure,
		    int32_t *temperature);

/* True once both a pressure and a temperature result are waiting. */
int dps310_data_ready(struct baro_dev *baro, bool *ready);

/*
 * The factory coefficients this part reported, for anyone wanting to do the
 * compensation themselves: c0, c1, c00, c10, c01, c11, c20, c21, c30.
 */
int dps310_coefficients(struct baro_dev *baro, int32_t out[9]);

/* Escape hatch for the registers this driver does not model. */
int dps310_reg_read(struct baro_dev *baro, uint8_t reg, void *buf, size_t len);
int dps310_reg_write(struct baro_dev *baro, uint8_t reg, uint8_t value);

#ifdef __cplusplus
}
#endif

#endif /* DPS310_H */
